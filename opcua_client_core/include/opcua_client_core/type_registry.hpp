#ifndef TYPE_REGISTRY_HPP
#define TYPE_REGISTRY_HPP

#include <any>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

#include "open62541pp/open62541pp.h"

template <typename T> struct TypeHolder
{
    using type = T;
    static const std::type_info &typeInfo() { return typeid(T); }
};

class TypeRegistry
{
  public:
    template <typename T> void registerType(const std::string &name)
    {
        typeGetters_[name] = []() -> const std::type_info & { return TypeHolder<T>::typeInfo(); };
        typeDispatchers_[name] = [](std::any &out) { return std::any_cast<opcua::Variant &>(out).getScalarCopy<T>(); };
    }

    const std::type_info &getType(const std::string &name) const
    {
        auto it = typeGetters_.find(name);
        if (it != typeGetters_.end())
        {
            return it->second();
        }
        else
        {
            return typeid(void);
        }
    }

    std::any dispatchType(const std::string &name, std::any &out) const
    {
        auto it = typeDispatchers_.find(name);
        if (it != typeDispatchers_.end())
        {
            return it->second(out);
        }
        else
        {
            throw std::runtime_error("Type not registered.");
        }
    }

  private:
    std::unordered_map<std::string, std::function<const std::type_info &()>> typeGetters_;
    std::unordered_map<std::string, std::function<std::any(std::any &)>> typeDispatchers_;
};

#endif // TYPE_REGISTRY_HPP
