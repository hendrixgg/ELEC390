# This file is used to extract the map representation from the map svg files.
# The map svg files are sourced from:
#   https://queensuca.sharepoint.com/teams/CCT-938446/SitePages/The-Town.aspx for "quackston-full-map.svg", "quackston-map-regions.svg", and "quackston-map-regions.svg"
#   https://queensuca.sharepoint.com/teams/CCT-938446/SitePages/GPS-and-Fare-System.aspx for "quackston-intersection-markings.svg"

# The svg files all appear to contain the same map content, but different layers are visible in each file.
# From the VPFS Server, the origin is at the bottom left corner of the map with y+ being upwards (opposite of svg) and x+ being rightwards (same as svg) (translated into svg coordinates, the origin is at (x=0.000cm, y=487.680cm)).
import xml.etree.ElementTree as ET
import re

tree = ET.parse('quackston-intersection-markings.svg')
root = tree.getroot()

vertices = []
for path in root.findall('.//{http://www.w3.org/2000/svg}path'):
    d = path.get('d')
    coords = re.findall(r'[ML]\s*(\d+\.?\d*),\s*(\d+\.?\d*)', d)
    for x, y in coords:
        vertices.append((float(x), float(y)))

print(vertices)  # Your raw vertex list
