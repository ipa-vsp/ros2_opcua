
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
struct Point
{

    float x;

    float y;

    float z;

} __attribute__((packed));

const opcua::DataType &getPointDataType()
{
    static const opcua::DataType dt = opcua::DataTypeBuilder<Point>::createStructure("Point", {1, 1}, {1, 4042})

                                          .addField<&Point::x>("x")

                                          .addField<&Point::y>("y")

                                          .addField<&Point::z>("z")

                                          .build();
    return dt;
}

/**
 * @brief This is a description
 */
struct Measurements
{

    opcua::String description;

    size_t mesurementSize;

    float *measurements;

} __attribute__((packed));

const opcua::DataType &getMeasurementsDataType()
{
    static const opcua::DataType dt =
        opcua::DataTypeBuilder<Measurements>::createStructure("Measurements", {1, 2}, {1, 4443})

            .addField<&Measurements::description>("description")

            .addField<&Measurements::mesurementSize>("mesurementSize")

            .addField<&Measurements::measurements>("measurements")

            .build();
    return dt;
}

/**
 * @brief Some description
 */
struct Opt
{

    int16_t a;

    float *b;

    float *c;

} __attribute__((packed));

const opcua::DataType &getOptDataType()
{
    static const opcua::DataType dt = opcua::DataTypeBuilder<Opt>::createStructure("Opt", {1, 3}, {1, 4644})

                                          .addField<&Opt::a>("a")

                                          .addField<&Opt::b>("b")

                                          .addField<&Opt::c>("c")

                                          .build();
    return dt;
}

struct NodeIdInfo
{
    uint16_t namespaceIndex;
    uint32_t identifier;
};

TypeRegistry struct_type_registry;

NodeIdInfo PointNodeIdInfo;

NodeIdInfo MeasurementsNodeIdInfo;

NodeIdInfo OptNodeIdInfo;

void registerTypes()
{

    struct_type_registry.registerType<Point>("Point");

    struct_type_registry.registerType<Measurements>("Measurements");

    struct_type_registry.registerType<Opt>("Opt");
}

namespace opcua
{

template <> struct TypeConverter<Point>
{
    using ValueType = Point;
    using NativeType = UA_ExtensionObject;
    using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

    static void fromNative(const NativeType &src, ValueType &dst)
    {
        // Check the encoding type
        if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
        {
            throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
        }

        PointNodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
        PointNodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

        if (src.content.encoded.body.length != sizeof(Point))
        {
            throw std::runtime_error("Size mismatch between Point and UA_ExtensionObject body length");
        }
        memcpy(&dst, src.content.encoded.body.data, sizeof(Point));
    }

    static void toNative(const ValueType &src, NativeType &dst)
    {
        // Allocate memory buffer
        if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof(Point)) != UA_STATUSCODE_GOOD)
        {
            throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
        }
        // Initialize memory
        memset(dst.content.encoded.body.data, 0, sizeof(Point));
        // Copy data
        memcpy(dst.content.encoded.body.data, &src, sizeof(Point));
        // Set the encoding type
        dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
        // Set Type ID (if necessary, example given below)
        dst.content.encoded.typeId = UA_NODEID_NUMERIC(PointNodeIdInfo.namespaceIndex, PointNodeIdInfo.identifier);
    }
};

template <> struct TypeConverter<Measurements>
{
    using ValueType = Measurements;
    using NativeType = UA_ExtensionObject;
    using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

    static void fromNative(const NativeType &src, ValueType &dst)
    {
        // Check the encoding type
        if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
        {
            throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
        }

        MeasurementsNodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
        MeasurementsNodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

        if (src.content.encoded.body.length != sizeof(Measurements))
        {
            throw std::runtime_error("Size mismatch between Measurements and UA_ExtensionObject body length");
        }
        memcpy(&dst, src.content.encoded.body.data, sizeof(Measurements));
    }

    static void toNative(const ValueType &src, NativeType &dst)
    {
        // Allocate memory buffer
        if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof(Measurements)) != UA_STATUSCODE_GOOD)
        {
            throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
        }
        // Initialize memory
        memset(dst.content.encoded.body.data, 0, sizeof(Measurements));
        // Copy data
        memcpy(dst.content.encoded.body.data, &src, sizeof(Measurements));
        // Set the encoding type
        dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
        // Set Type ID (if necessary, example given below)
        dst.content.encoded.typeId =
            UA_NODEID_NUMERIC(MeasurementsNodeIdInfo.namespaceIndex, MeasurementsNodeIdInfo.identifier);
    }
};

template <> struct TypeConverter<Opt>
{
    using ValueType = Opt;
    using NativeType = UA_ExtensionObject;
    using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

    static void fromNative(const NativeType &src, ValueType &dst)
    {
        // Check the encoding type
        if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
        {
            throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
        }

        OptNodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
        OptNodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

        if (src.content.encoded.body.length != sizeof(Opt))
        {
            throw std::runtime_error("Size mismatch between Opt and UA_ExtensionObject body length");
        }
        memcpy(&dst, src.content.encoded.body.data, sizeof(Opt));
    }

    static void toNative(const ValueType &src, NativeType &dst)
    {
        // Allocate memory buffer
        if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof(Opt)) != UA_STATUSCODE_GOOD)
        {
            throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
        }
        // Initialize memory
        memset(dst.content.encoded.body.data, 0, sizeof(Opt));
        // Copy data
        memcpy(dst.content.encoded.body.data, &src, sizeof(Opt));
        // Set the encoding type
        dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
        // Set Type ID (if necessary, example given below)
        dst.content.encoded.typeId = UA_NODEID_NUMERIC(OptNodeIdInfo.namespaceIndex, OptNodeIdInfo.identifier);
    }
};

} // namespace opcua

#endif // DATA_TYPES_H_
