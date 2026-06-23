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

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"
#include "comm/datalayer/memory_map_generated.h"

struct SharedMemoryVariable
{
  std::string name;
  std::string type;

  uint32_t bit_offset = 0;  // Offset in bits
  uint32_t bit_size = 0;    // Size in bits

  // Rightshift by 3 is equal to deviding by 8, gives the byte offset
  uint32_t byte_index() const
  {
    return bit_offset >> 3;
  }
  // And with 7 gives the reminder of division by 8, gives the bit offset
  uint8_t bit_index() const
  {
    return bit_offset & 7;
  }

  bool is_byte_aligned() const
  {
    return (bit_offset & 7) == 0;
  }

  double value = std::numeric_limits<double>::quiet_NaN();
  bool uptodate = false;
};

class SharedMemoryArea
{
public:
  SharedMemoryArea(
    comm::datalayer::DatalayerSystem * datalayerSystem,
    comm::datalayer::IClient * client,
    std::string address)
  : datalayerSystem_(datalayerSystem),
    client_(client),
    address_(std::move(address))
  {
  }

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
      log << "Variable added " << var.name << ", ";
      log << var.bit_offset << ", ";
      log << var.bit_size << ", ";
      log << var.byte_index() << ", ";
      log << static_cast<unsigned>(var.bit_index()) << "\n";
    }

    if (!(variables_.size() > 0)) {
      what = "Failed to refresh memory map: no variables found";
      return comm::datalayer::DlResult::DL_FAILED;
    }

    what = log.str();
    return comm::datalayer::DlResult::DL_OK;
  }

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

private:
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
