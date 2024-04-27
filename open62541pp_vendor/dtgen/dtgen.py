import sys
import yaml
import jinja2
import os
from ament_index_python import get_package_share_directory

def generate_header(yaml_path, output_dir):
    # Load YAML file
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)

    # Define the inline Jinja2 template
    template_string = """
    #ifndef GENERATED_DATATYPES_HPP
    #define GENERATED_DATATYPES_HPP

    {% for var_name, var_details in data.items() %}
    struct {{ var_name }}
    {
        {% for element in var_details.elements %}
        {{ element.type }} {{ element.name }};
        {% endfor %}
    };
    {% endfor %}

    #endif // GENERATED_DATATYPES_HPP
    """
    template = jinja2.Template(template_string)

    # Render template with data from YAML
    output = template.render(data=data)

    # Write the output to a header file
    header_filename = f"{output_dir}"
    with open(header_filename, 'w') as header_file:
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
