#ifndef SHARED_MEMORY_HELPER_HPP
#define  SHARED_MEMORY_HELPER_HPP

#include <string>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <utility>
#include <thread>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"
#include "comm/datalayer/memory_map_generated.h"

/**
 * @brief Metadata and cached value for one ctrlX Data Layer shared-memory variable.
 *
 * The memory map reports variable offsets and sizes in bits. This helper keeps
 * the original bit-level metadata, exposes byte/bit convenience accessors, and
 * stores the last value read from or staged for the shared-memory area.
 */
struct SharedMemoryVariable
{
  /// Variable name from the Data Layer memory map.
  std::string name;

  /// Data Layer PLC type string, for example comm::datalayer::TYPE_PLC_DINT.
  std::string type;

  /// Offset of the variable from the beginning of the memory area, in bits.
  uint32_t bit_offset = 0;  // Offset in bits

  /// Size of the variable in bits.
  uint32_t bit_size = 0;    // Size in bits

  /**
   * @brief Return the byte index that contains the start of this variable.
   * @return Zero-based byte offset from the beginning of the memory area.
   */
  uint32_t byte_index() const
  {
    return bit_offset >> 3;
  }

  /**
   * @brief Return the bit index within the starting byte.
   * @return Bit offset in the range [0, 7].
   */
  uint8_t bit_index() const
  {
    return bit_offset & 7;
  }

  /**
   * @brief Check whether the variable starts on a byte boundary.
   * @return True when bit_offset is evenly divisible by eight.
   */
  bool is_byte_aligned() const
  {
    return (bit_offset & 7) == 0;
  }

  /// Last cached numeric value for this variable.
  double value = std::numeric_limits<double>::quiet_NaN();

  /// True when value matches the last successful read or write operation.
  bool uptodate = false;
};

/**
 * @brief Access helper for one ctrlX Data Layer shared-memory area.
 *
 * SharedMemoryArea reads the Data Layer memory map, opens the corresponding
 * shared-memory block, caches variable metadata by name, and provides typed
 * read/write helpers for PLC DINT variables. Each operation reports a
 * comm::datalayer::DlResult and writes a human-readable status or error message
 * into the supplied @p what string.
 */
class SharedMemoryArea
{
public:
  /**
   * @brief Create a helper for a Data Layer shared-memory area.
   * @param datalayerSystem Data Layer system used to open the memory user.
   * @param client Connected Data Layer client used to read the memory map.
   * @param address Base Data Layer address of the shared-memory area.
   *
   * The caller owns @p datalayerSystem and @p client and must keep both alive
   * for the lifetime of this helper.
   */
  SharedMemoryArea(
    comm::datalayer::DatalayerSystem * datalayerSystem,
    comm::datalayer::IClient * client,
    std::string address)
  : datalayerSystem_(datalayerSystem),
    client_(client),
    address_(std::move(address))
  {
  }

  /**
   * @brief Refresh the memory map and rebuild the local variable cache.
   * @param[out] what Status or error details for logging.
   * @return DL_OK when the map is loaded or already current; otherwise the
   * Data Layer error that prevented the refresh.
   *
   * This method reads @c address_ plus @c "/map", opens the shared-memory area
   * on first use, verifies the flatbuffer payload, retries getMemoryMap()
   * briefly, and updates the cached revision and variable table when the map
   * revision changes.
   */
  comm::datalayer::DlResult refresh_map(std::string & what)
  {
    comm::datalayer::Variant variantMap;
    std::string mapAddress = address_ + "/map";

    auto res = client_->readSync(mapAddress, &variantMap);
    if (STATUS_FAILED(res)) {
      what = "Failed to refresh memory map: failed to read " + mapAddress;
      return res;
    }
    if (!memoryUser_) {
      res = datalayerSystem_->factory()->openMemory(memoryUser_, address_);
      if (STATUS_FAILED(res)) {
        what = "Failed to refresh memory map: failed to open memory";
        return res;
      }
    }

    res = variantMap.verifyFlatbuffers(comm::datalayer::VerifyMemoryMapBuffer);
    if (STATUS_FAILED(res)) {
      what = "Failed to refresh memory map: flatbuffers verification failed";
      return res;
    }

    constexpr int max_attempts = 10;
    int current_attempt = 0;
    do{
      res = memoryUser_->getMemoryMap(variantMap);

      if (STATUS_SUCCEEDED(res)) {
        break;
      }

      std::cout
        << "getMemoryMap attempt " << current_attempt << "/"
        << max_attempts << " failed: "
        << res.toString()
        << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      current_attempt++;
    }while(current_attempt < max_attempts);

    if (STATUS_FAILED(res)) {
      what = "Failed to refresh memory map: failed to get memory map";
      return res;
    }

    auto memoryMap = comm::datalayer::GetMemoryMap(variantMap.getData());
    auto tempRev = memoryMap->revision();

    if (tempRev == revision_ && revision_ != 0) {
      what = "Memory map already up to date";
      return comm::datalayer::DlResult::DL_OK;
    }

    revision_ = tempRev;
    variables_.clear();

    std::stringstream log;
    log << "Mapping datalayer variables at " << mapAddress << "\n";
    for (auto variable = memoryMap->variables()->begin();
      variable != memoryMap->variables()->end(); variable++)
    {
      SharedMemoryVariable var;
      var.name = variable->name()->str();
      var.bit_offset = variable->bitoffset();
      var.bit_size = variable->bitsize();
      var.type = variable->type()->str();
      variables_.emplace(var.name, var);
      log << "Variable added ";
      log << "[name: " << var.name << ", ";
      log << "bit offset: " << var.bit_offset << ", ";
      log << "bit size: " << var.bit_size << ", ";
      log << "byte_index: " << var.byte_index() << ", ";
      log << "bit_index: " << static_cast<unsigned>(var.bit_index()) << "]\n";
    }

    if (!(variables_.size() > 0)) {
      what = "Failed to refresh memory map: no variables found at " + mapAddress;
      return comm::datalayer::DlResult::DL_FAILED;
    }

    what = log.str();
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Read all supported variables from shared memory into the cache.
   * @param[out] what Status or error details for logging.
   * @return DL_OK when all supported variables are read successfully; otherwise
   * the first Data Layer or validation error encountered.
   *
   * Currently only PLC DINT variables are read. Unsupported variable types are
   * skipped.
   */
  comm::datalayer::DlResult readVariables(std::string & what)
  {
    uint8_t * firstBytePtr;
    auto res = memoryUser_->beginAccess(firstBytePtr, revision_);
    if (STATUS_FAILED(res)) {
      what = "Failed to access variables at " + address_;
      return res;
    }

    for (auto & [name, var] : variables_) {

      // For the moment i am just interested in reading
      if (var.type == comm::datalayer::TYPE_PLC_DINT) {
        res = readVariableFromMemory(var, firstBytePtr, what);
        if (STATUS_FAILED(res)) {
          memoryUser_->endAccess();
          return res;
        }
      }
    }
    memoryUser_->endAccess();
    what = "Variables read.";
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Write all supported cached variables to shared memory.
   * @param[out] what Status or error details for logging.
   * @return DL_OK when all supported variables are written successfully;
   * otherwise the first Data Layer or validation error encountered.
   *
   * Currently only PLC DINT variables are written. Unsupported variable types
   * are skipped.
   */
  comm::datalayer::DlResult writeVariables(std::string & what)
  {
    uint8_t * firstBytePtr;
    auto res = memoryUser_->beginAccess(firstBytePtr, revision_);
    if (STATUS_FAILED(res)) {
      what = "Failed to access variables at " + address_;
      return res;
    }

    for (auto & [name, var] : variables_) {

      if (var.type == comm::datalayer::TYPE_PLC_DINT) {
        res = writeVariableToMemory(var, firstBytePtr, what);
        if (STATUS_FAILED(res)) {
          memoryUser_->endAccess();
          return res;
        }
      }
    }
    what = "Variables successfulyy written";
    memoryUser_->endAccess();
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Read one named variable directly from shared memory.
   * @param[in] name Variable name as reported by the memory map.
   * @param[out] value Value read from shared memory.
   * @param[out] what Status or error details for logging.
   * @return DL_OK on success, DL_INVALID_ADDRESS when @p name is unknown, or a
   * Data Layer/validation error from the access attempt.
   *
   * The cached value for @p name is updated when the read succeeds.
   */
  comm::datalayer::DlResult readVariable(
    const std::string & name, double & value, std::string & what)
  {
    auto var = variables_.find(name);
    if (var == variables_.end()) {
      what = "Variable not found in " + address_ + ": " + name;
      return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
    }

    uint8_t * firstBytePtr;
    auto res = memoryUser_->beginAccess(firstBytePtr, revision_);
    if (STATUS_FAILED(res)) {
      what = "Failed to access variable at " + address_ + ": " + name;
      return res;
    }

    res = readVariableFromMemory(var->second, firstBytePtr, what);
    memoryUser_->endAccess();
    if (STATUS_FAILED(res)) {
      return res;
    }

    value = var->second.value;
    what = "Variable read: " + name;
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Write one named variable directly to shared memory.
   * @param[in] name Variable name as reported by the memory map.
   * @param[in] value Numeric value to write. PLC DINT values are rounded and
   * range-checked before writing.
   * @param[out] what Status or error details for logging.
   * @return DL_OK on success, DL_INVALID_ADDRESS when @p name is unknown, or a
   * Data Layer/validation error from the access attempt.
   *
   * The cached value for @p name is updated before the write attempt and marked
   * up to date when the write succeeds.
   */
  comm::datalayer::DlResult writeVariable(
    const std::string & name, double value, std::string & what)
  {
    auto var = variables_.find(name);
    if (var == variables_.end()) {
      what = "Variable not found in " + address_ + ": " + name;
      return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
    }

    var->second.value = value;
    var->second.uptodate = false;

    uint8_t * firstBytePtr;
    auto res = memoryUser_->beginAccess(firstBytePtr, revision_);
    if (STATUS_FAILED(res)) {
      what = "Failed to access variable at " + address_ + ": " + name;
      return res;
    }

    res = writeVariableToMemory(var->second, firstBytePtr, what);
    memoryUser_->endAccess();
    if (STATUS_FAILED(res)) {
      return res;
    }

    what = "Variable written: " + name;
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Return the cached value for a named variable without reading memory.
   * @param[in] name Variable name as reported by the memory map.
   * @param[out] value Cached value for @p name.
   * @param[out] what Status or error details for logging.
   * @return DL_OK on success or DL_INVALID_ADDRESS when @p name is unknown.
   */
  comm::datalayer::DlResult getVariableValue(
    const std::string & name, double & value, std::string & what) const
  {
    auto var = variables_.find(name);
    if (var == variables_.end()) {
      what = "Variable not found in " + address_ + ": " + name;
      return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
    }

    value = var->second.value;
    what = "Variable value retrieved: " + name;
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Stage a cached value for a named variable without writing memory.
   * @param[in] name Variable name as reported by the memory map.
   * @param[in] value Value to cache for a later writeVariables() call.
   * @param[out] what Status or error details for logging.
   * @return DL_OK on success or DL_INVALID_ADDRESS when @p name is unknown.
   */
  comm::datalayer::DlResult setVariableValue(
    const std::string & name, double value, std::string & what)
  {
    auto var = variables_.find(name);
    if (var == variables_.end()) {
      what = "Variable not found in " + address_ + ": " + name;
      return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
    }

    var->second.value = value;
    var->second.uptodate = false;
    what = "Variable value set: " + name;
    return comm::datalayer::DlResult::DL_OK;
  }

  std::unordered_map<std::string, SharedMemoryVariable> getVariables()
  {
    return variables_;
  }

private:
  /**
   * @brief Read one validated PLC DINT from an active memory access window.
   * @param[in,out] var Variable metadata and cached value to update.
   * @param[in] firstBytePtr Pointer returned by IMemoryUser::beginAccess().
   * @param[out] what Error details when validation fails.
   * @return DL_OK on success or a validation error.
   */
  comm::datalayer::DlResult readVariableFromMemory(
    SharedMemoryVariable & var, const uint8_t * firstBytePtr, std::string & what)
  {
    auto res = validateDintVariable(var, "read", what);
    if (STATUS_FAILED(res)) {
      return res;
    }

    int32_t raw = 0;
    std::memcpy(&raw, firstBytePtr + var.byte_index(), sizeof(raw));
    var.value = static_cast<double>(raw);
    var.uptodate = true;
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Write one validated PLC DINT into an active memory access window.
   * @param[in,out] var Variable metadata and cached value to write.
   * @param[in,out] firstBytePtr Pointer returned by IMemoryUser::beginAccess().
   * @param[out] what Error details when validation or conversion fails.
   * @return DL_OK on success or a validation/conversion error.
   */
  comm::datalayer::DlResult writeVariableToMemory(
    SharedMemoryVariable & var, uint8_t * firstBytePtr, std::string & what)
  {
    auto res = validateDintVariable(var, "write", what);
    if (STATUS_FAILED(res)) {
      return res;
    }

    if (!std::isfinite(var.value)) {
      what = "Invalid DINT value for " + var.name + ": not finite";
      return comm::datalayer::DlResult::DL_TYPE_MISMATCH;
    }

    const double roundedValue = std::round(var.value);
    if (roundedValue > static_cast<double>(std::numeric_limits<int32_t>::max())) {
      what = "Invalid DINT value for " + var.name + ": max exceeded";
      return comm::datalayer::DlResult::DL_LIMIT_MAX;
    }

    if (roundedValue < static_cast<double>(std::numeric_limits<int32_t>::min())) {
      what = "Invalid DINT value for " + var.name + ": min exceeded";
      return comm::datalayer::DlResult::DL_LIMIT_MIN;
    }

    const int32_t val = static_cast<int32_t>(roundedValue);
    std::memcpy(firstBytePtr + var.byte_index(), &val, sizeof(val));
    var.uptodate = true;
    return comm::datalayer::DlResult::DL_OK;
  }

  /**
   * @brief Validate that a mapped variable can be accessed as a PLC DINT.
   * @param[in] var Variable metadata to validate.
   * @param[in] operation Operation name used in error messages.
   * @param[out] what Error details when validation fails.
   * @return DL_OK when @p var is a byte-aligned 32-bit PLC DINT; otherwise a
   * Data Layer validation error.
   */
  comm::datalayer::DlResult validateDintVariable(
    const SharedMemoryVariable & var, const std::string & operation, std::string & what) const
  {
    if (var.type != comm::datalayer::TYPE_PLC_DINT) {
      what = "Failed to " + operation + " variable from " + address_ + ": " + var.name +
        " unsupported type.";
      return comm::datalayer::DlResult::DL_TYPE_MISMATCH;
    }

    if (!var.is_byte_aligned() || var.bit_size != 32) {
      what = "Failed to " + operation + " variable from " + address_ + ": " + var.name +
        " wrong configuration.";
      return comm::datalayer::DlResult::DL_INVALID_CONFIGURATION;
    }

    return comm::datalayer::DlResult::DL_OK;
  }

  comm::datalayer::DatalayerSystem * datalayerSystem_ = nullptr;
  comm::datalayer::IClient * client_ = nullptr;
  std::string address_;
  uint32_t revision_ = 0;
  std::shared_ptr<comm::datalayer::IMemoryUser> memoryUser_;
  std::unordered_map<std::string, SharedMemoryVariable> variables_;
};
#endif
