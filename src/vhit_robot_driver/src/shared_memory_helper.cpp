#include "shared_memory_helper.hpp"

SharedMemoryArea::SharedMemoryArea(
  comm::datalayer::DatalayerSystem * datalayerSystem,
  comm::datalayer::IClient * client,
  std::string address)
: datalayerSystem_(datalayerSystem),
  client_(client),
  address_(std::move(address))
{
}

comm::datalayer::DlResult SharedMemoryArea::refresh_map(std::string & what)
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

  what = log.str();
  return comm::datalayer::DlResult::DL_OK;
}

comm::datalayer::DlResult SharedMemoryArea::readVariables(std::string & what)
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

comm::datalayer::DlResult SharedMemoryArea::writeVariables(std::string & what, bool force)
{
  uint8_t * firstBytePtr;
  auto res = memoryUser_->beginAccess(firstBytePtr, revision_);
  if (STATUS_FAILED(res)) {
    what = "Failed to access variables at " + address_;
    return res;
  }

  for (auto & [name, var] : variables_) {

    if (var.type == comm::datalayer::TYPE_PLC_DINT &&
      (!var.updated || force) &&
      !std::isnan(var.value))
    {
      res = writeVariableToMemory(var, firstBytePtr, what);
      if (STATUS_FAILED(res)) {
        memoryUser_->endAccess();
        return res;
      }
    }
  }
  what = "Variables successfully written";
  memoryUser_->endAccess();
  return comm::datalayer::DlResult::DL_OK;
}

comm::datalayer::DlResult SharedMemoryArea::readVariable(
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

comm::datalayer::DlResult SharedMemoryArea::writeVariable(
  const std::string & name, double value, std::string & what)
{
  auto var = variables_.find(name);
  if (var == variables_.end()) {
    what = "Variable not found in " + address_ + ": " + name;
    return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
  }

  var->second.value = value;
  var->second.updated = false;

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

comm::datalayer::DlResult SharedMemoryArea::getVariableValue(
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

comm::datalayer::DlResult SharedMemoryArea::setVariableValue(
  const std::string & name, double value, std::string & what)
{
  auto var = variables_.find(name);
  if (var == variables_.end()) {
    what = "Variable not found in " + address_ + ": " + name;
    return comm::datalayer::DlResult::DL_INVALID_ADDRESS;
  }

  var->second.value = value;
  var->second.updated = false;
  what = "Variable value set: " + name;
  return comm::datalayer::DlResult::DL_OK;
}

std::unordered_map<std::string, SharedMemoryVariable> SharedMemoryArea::getVariables()
{
  return variables_;
}

const comm::datalayer::DlResult SharedMemoryArea::beginAccessRt(uint8_t * & firstBytePtr)
{
  firstBytePtr = DL_RT_NON_BLOCKING;
  return memoryUser_->beginAccess(firstBytePtr, revision_);
}

const comm::datalayer::DlResult SharedMemoryArea::endAccessRt()
{
  return memoryUser_->endAccess();
}

void SharedMemoryArea::readVariableRt(
  const uint32_t byteIndex, const uint8_t * firstBytePtr, int32_t & val)
{
  std::memcpy(&val, firstBytePtr + byteIndex, sizeof(val));
}

void SharedMemoryArea::writeVariableRt(
  const uint32_t byteIndex, uint8_t * firstBytePtr, const int32_t & val)
{
  std::memcpy(firstBytePtr + byteIndex, &val, sizeof(val));
}

comm::datalayer::DlResult SharedMemoryArea::validateDintVariable(
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

comm::datalayer::DlResult SharedMemoryArea::readVariableFromMemory(
  SharedMemoryVariable & var, const uint8_t * firstBytePtr, std::string & what)
{
  auto res = validateDintVariable(var, "read", what);
  if (STATUS_FAILED(res)) {
    return res;
  }

  int32_t raw = 0;
  std::memcpy(&raw, firstBytePtr + var.byte_index(), sizeof(raw));
  var.value = static_cast<double>(raw);
  var.updated = true;
  return comm::datalayer::DlResult::DL_OK;
}

comm::datalayer::DlResult SharedMemoryArea::writeVariableToMemory(
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

  if (std::isnan(var.value)) {
    what = "Invalid DINT value for " + var.name + ": not a number";
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
  var.updated = true;
  return comm::datalayer::DlResult::DL_OK;
}
