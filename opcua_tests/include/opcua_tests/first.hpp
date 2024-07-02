
#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include "open62541pp/open62541pp.h"
#include <cstdint>
#include <iostream>

#include "opcua_client_core/type_registry.hpp"

// Additional includes
#include "open62541/client_config_default.h"
#include "open62541/types.h"
#include "open62541/types_generated.h"
#include "open62541/types_generated_handling.h"


/**
 * @brief This is a description
 */
struct variable1
{
    
    int name;
    
    bool name1;
    
} __attribute__((packed));

const opcua::DataType &getvariable1DataType()
{
    static const opcua::DataType dt =
        opcua::DataTypeBuilder<variable1>::createStructure("variable1", {1, 1}, {1, 4023 })
            
            .addField<&variable1::name>("name")
            
            .addField<&variable1::name1>("name1")
            
            .build();
    return dt;
}


/**
 * @brief This is a description
 */
struct variable2
{
    
    int name;
    
    bool name1;
    
} __attribute__((packed));

const opcua::DataType &getvariable2DataType()
{
    static const opcua::DataType dt =
        opcua::DataTypeBuilder<variable2>::createStructure("variable2", {1, 2}, {1, 4024 })
            
            .addField<&variable2::name>("name")
            
            .addField<&variable2::name1>("name1")
            
            .build();
    return dt;
}



struct NodeIdInfo
{
    uint16_t namespaceIndex;
    uint32_t identifier;
};

TypeRegistry struct_type_registry;


NodeIdInfo variable1NodeIdInfo;

NodeIdInfo variable2NodeIdInfo;


void registerTypes()
{
    
    struct_type_registry.registerType<variable1>("variable1");
    
    struct_type_registry.registerType<variable2>("variable2");
    
}

namespace opcua {
    
    template <>
    struct TypeConverter<variable1>
    {
        using ValueType = variable1;
        using NativeType = UA_ExtensionObject;
        using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

        static void fromNative(const NativeType &src, ValueType &dst)
        {
            // Check the encoding type
            if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
            {
                throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
            }

            variable1NodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
            variable1NodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

            if (src.content.encoded.body.length != sizeof(variable1))
            {
                throw std::runtime_error("Size mismatch between variable1 and UA_ExtensionObject body length");
            }
            memcpy(&dst, src.content.encoded.body.data, sizeof(variable1));
        }

        static void toNative(const ValueType &src, NativeType &dst)
        {
            // Allocate memory buffer
            if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof(variable1)) != UA_STATUSCODE_GOOD)
            {
                throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
            }
            // Initialize memory
            memset(dst.content.encoded.body.data, 0, sizeof(variable1));
            // Copy data
            memcpy(dst.content.encoded.body.data, &src, sizeof(variable1));
            // Set the encoding type
            dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
            // Set Type ID (if necessary, example given below)
            dst.content.encoded.typeId = UA_NODEID_NUMERIC(variable1NodeIdInfo.namespaceIndex, variable1NodeIdInfo.identifier);
        }
    };
    
    template <>
    struct TypeConverter<variable2>
    {
        using ValueType = variable2;
        using NativeType = UA_ExtensionObject;
        using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

        static void fromNative(const NativeType &src, ValueType &dst)
        {
            // Check the encoding type
            if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
            {
                throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
            }

            variable2NodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
            variable2NodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

            if (src.content.encoded.body.length != sizeof(variable2))
            {
                throw std::runtime_error("Size mismatch between variable2 and UA_ExtensionObject body length");
            }
            memcpy(&dst, src.content.encoded.body.data, sizeof(variable2));
        }

        static void toNative(const ValueType &src, NativeType &dst)
        {
            // Allocate memory buffer
            if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof(variable2)) != UA_STATUSCODE_GOOD)
            {
                throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
            }
            // Initialize memory
            memset(dst.content.encoded.body.data, 0, sizeof(variable2));
            // Copy data
            memcpy(dst.content.encoded.body.data, &src, sizeof(variable2));
            // Set the encoding type
            dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
            // Set Type ID (if necessary, example given below)
            dst.content.encoded.typeId = UA_NODEID_NUMERIC(variable2NodeIdInfo.namespaceIndex, variable2NodeIdInfo.identifier);
        }
    };
    
}

#endif // DATA_TYPES_H_
    