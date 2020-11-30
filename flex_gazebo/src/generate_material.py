# Import the element creator
from pcg_gazebo.parsers.sdf import create_sdf_element

# Material elements are initially empty since all its child elements are optional
material = create_sdf_element('material')
# To create all optional elements with its default elements, use reset()
material.reset(with_optional_elements=True)
print(material)