#ifndef TYPE_REGISTRY_HPP
#define TYPE_REGISTRY_HPP

#include <iostream>
#include <string>
#include <unordered_map>
#include <functional>

template <typename T>
struct TypeHolder
{
    using type = T;
};

// Custom implementation of type_identity for C++17 compatibility
template <typename T>
struct type_identity
{
    using type = T;
};

template <typename T>
using type_identity_t = typename type_identity<T>::type;

class TypeRegistry
{
public:
    using TypeCreatorFunc = std::function<const std::type_info&()>;

    template <typename T>
    void registerType(const std::string& name)
    {
        typeGetters_[name] = []() -> const std::type_info& { return typeid(T); };
    }

    template <typename DefaultType>
    const std::type_info& getType(const std::string& name) const
    {
        auto it = typeGetters_.find(name);
        if (it != typeGetters_.end())
        {
            return it->second();
        }
        else
        {
            return typeid(DefaultType);
        }
    }

private:
    std::unordered_map<std::string, TypeCreatorFunc> typeGetters_;
};

#endif // TYPE_REGISTRY_HPP
