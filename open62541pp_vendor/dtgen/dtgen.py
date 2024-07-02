import sys
import yaml
import jinja2
# import os
# from ament_index_python import get_package_share_directory


def gen_template_string():
    template = """
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

{% for struct_name, struct_details in data.variables.items() if struct_details.type == 'struct' %}
/**
 * @brief {{ struct_details.description }}
 */
struct {{ struct_name }}
{
    {% for element in struct_details.elements %}
    {{ element.type }} {{ element.name }};
    {% endfor %}
} __attribute__((packed));

const opcua::DataType &get{{ struct_name }}DataType()
{
    static const opcua::DataType dt =
        opcua::DataTypeBuilder<{{ struct_name }}>::createStructure("{{ struct_name }}", {{ struct_details.typeID }}, {{ struct_details.bynaryTypeID }})
            {% for element in struct_details.elements %}
            .addField<&{{ struct_name }}::{{ element.name }}>("{{ element.name }}")
            {% endfor %}
            .build();
    return dt;
}

{% endfor %}

struct NodeIdInfo
{
    uint16_t namespaceIndex;
    uint32_t identifier;
};

TypeRegistry struct_type_registry;

{% for struct_name, struct_details in data.variables.items() if struct_details.type == 'struct' %}
NodeIdInfo {{ struct_name }}NodeIdInfo;
{% endfor %}

void registerTypes()
{
    {% for struct_name, struct_details in data.variables.items() if struct_details.type == 'struct' %}
    struct_type_registry.registerType<{{ struct_name }}>("{{ struct_name}}");
    {% endfor %}
}

namespace opcua {
    {% for struct_name, struct_details in data.variables.items() if struct_details.type == 'struct' %}
    template <>
    struct TypeConverter<{{ struct_name }}>
    {
        using ValueType = {{ struct_name }};
        using NativeType = UA_ExtensionObject;
        using ValidTypes = TypeIndexList<UA_TYPES_EXTENSIONOBJECT>;

        static void fromNative(const NativeType &src, ValueType &dst)
        {
            // Check the encoding type
            if (src.encoding != UA_EXTENSIONOBJECT_ENCODED_BYTESTRING)
            {
                throw std::runtime_error("Invalid encoding type for UA_ExtensionObject");
            }

            {{ struct_name }}NodeIdInfo.namespaceIndex = src.content.encoded.typeId.namespaceIndex;
            {{ struct_name }}NodeIdInfo.identifier = src.content.encoded.typeId.identifier.numeric;

            if (src.content.encoded.body.length != sizeof({{ struct_name }}))
            {
                throw std::runtime_error("Size mismatch between {{ struct_name }} and UA_ExtensionObject body length");
            }
            memcpy(&dst, src.content.encoded.body.data, sizeof({{ struct_name }}));
        }

        static void toNative(const ValueType &src, NativeType &dst)
        {
            // Allocate memory buffer
            if (UA_ByteString_allocBuffer(&dst.content.encoded.body, sizeof({{ struct_name }})) != UA_STATUSCODE_GOOD)
            {
                throw std::runtime_error("Failed to allocate memory for UA_ExtensionObject");
            }
            // Initialize memory
            memset(dst.content.encoded.body.data, 0, sizeof({{ struct_name }}));
            // Copy data
            memcpy(dst.content.encoded.body.data, &src, sizeof({{ struct_name }}));
            // Set the encoding type
            dst.encoding = UA_EXTENSIONOBJECT_ENCODED_BYTESTRING;
            // Set Type ID (if necessary, example given below)
            dst.content.encoded.typeId = UA_NODEID_NUMERIC({{ struct_name }}NodeIdInfo.namespaceIndex, {{ struct_name }}NodeIdInfo.identifier);
        }
    };
    {% endfor %}
}

#endif // DATA_TYPES_H_
    """
    return template


def generate_header(yaml_path, output_dir):
    # Load YAML file
    with open(yaml_path) as file:
        data = yaml.safe_load(file)

    # Define the inline Jinja2 template
    template_string = gen_template_string()
    template = jinja2.Template(template_string)

    # Render template with data from YAML
    output = template.render(data=data)

    # Write the output to a header file
    header_filename = f"{output_dir}"
    with open(header_filename, "w") as header_file:
        header_file.write(output)

    print(f"Header file generated at {header_filename}")


def main():
    if len(sys.argv) != 3:
        print("Usage: python dtgen.py <YAML_FILE> <OUTPUT_DIR>")
        sys.exit(1)

    yaml_file = sys.argv[1]
    output_dir = sys.argv[2]
    try:
        generate_header(yaml_file, output_dir)
    except Exception as e:
        print(f"Error generating headers: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
