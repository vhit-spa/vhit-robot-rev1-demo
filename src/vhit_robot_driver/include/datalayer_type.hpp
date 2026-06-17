// Copyright (c) 2025 B-ROBOTIZED GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#include <cmath>
#include <cstdint>
#include <limits>

class DatalayerType
{
public:
  explicit DatalayerType(
    const double & value, const comm::datalayer::VariantType & type,
    const std::string & address)
  : value(value), type_(type), address_(address)
  {
    if (!is_supported_type(type_)) {
      throw std::invalid_argument(
              "Failed to initialize the DatalayerType because the type you passed is not supported.");
    }
    if (address_.empty()) {
      throw std::invalid_argument(
              "Failed to initialize the DatalayerType because of an empty address.");
    }
  }

  bool set_type(const comm::datalayer::VariantType & variant_type)
  {
    if (is_supported_type(variant_type)) {
      type_ = variant_type;
      return true;
    }
    return false;
  }

  comm::datalayer::VariantType get_type() const
  {
    return type_;
  }

  const std::string & address() const {return address_;}

  comm::datalayer::Variant getValueAsDatalayerVariant() const
  {
    comm::datalayer::Variant variant;

    switch (type_) {
      case comm::datalayer::VariantType::BOOL8:
        {
          variant.setValue(static_cast<bool>(value));
          break;
        }
      case comm::datalayer::VariantType::INT32:
        {
          constexpr auto int32_max = std::numeric_limits<int32_t>::max();
          constexpr auto int32_min = std::numeric_limits<int32_t>::min();
          constexpr auto int32_max_as_double = static_cast<double>(int32_max);
          constexpr auto int32_min_as_double = static_cast<double>(int32_min);

          if (value >= int32_max_as_double) {
            variant.setValue(int32_max);
          } else if (value <= int32_min_as_double) {
            variant.setValue(int32_min);
          } else {
            variant.setValue(static_cast<int32_t>(std::round(value)));
          }
          break;
        }
      case comm::datalayer::VariantType::INT64:
        {
          constexpr auto int64_max = std::numeric_limits<int64_t>::max();
          constexpr auto int64_min = std::numeric_limits<int64_t>::min();
          constexpr auto int64_max_as_double = static_cast<double>(int64_max);
          constexpr auto int64_min_as_double = static_cast<double>(int64_min);

          if (value >= int64_max_as_double) {
            variant.setValue(int64_max);
          } else if (value <= int64_min_as_double) {
            variant.setValue(int64_min);
          } else {
            variant.setValue(static_cast<int64_t>(std::round(value)));
          }
          break;
        }
      case comm::datalayer::VariantType::FLOAT64:
        {
          variant.setValue(value);
          break;
        }
      default:
        {
          throw std::runtime_error(
                  "Failed to convert to datalayer VariantType because of internal error. The set typ is not supported. Should never happen.");
          break;
        }
    }
    return variant;
  }

  double value;

protected:
  bool is_supported_type(const comm::datalayer::VariantType & variant_type)
  {
    if (variant_type == comm::datalayer::VariantType::BOOL8 ||
      variant_type == comm::datalayer::VariantType::INT32 ||
      variant_type == comm::datalayer::VariantType::INT64 ||
      variant_type == comm::datalayer::VariantType::UINT64 ||
      variant_type == comm::datalayer::VariantType::FLOAT64)
    {
      return true;
    }
    return false;
  }

private:
  comm::datalayer::VariantType type_;
  const std::string address_;
};
