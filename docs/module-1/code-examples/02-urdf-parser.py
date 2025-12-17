#!/usr/bin/env python3
"""
Simple URDF Parser Example
Parses a URDF file and extracts joint and link information.
"""

import xml.etree.ElementTree as ET


class URDFParser:
    """A simple URDF parser for extracting robot structure."""

    def __init__(self, urdf_file_path):
        """Initialize parser with URDF file path."""
        self.urdf_file = urdf_file_path
        self.tree = ET.parse(urdf_file_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get('name', 'unknown')

    def get_links(self):
        """Extract all links from the URDF."""
        links = []
        for link in self.root.findall('link'):
            link_name = link.get('name')
            links.append(link_name)
        return links

    def get_joints(self):
        """Extract all joints from the URDF."""
        joints = []
        for joint in self.root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            joints.append({
                'name': joint_name,
                'type': joint_type,
                'parent': parent,
                'child': child
            })
        return joints

    def print_structure(self):
        """Print the robot structure in human-readable format."""
        print(f"Robot Name: {self.robot_name}\n")

        links = self.get_links()
        print(f"Links ({len(links)}):")
        for link in links:
            print(f"  - {link}")

        joints = self.get_joints()
        print(f"\nJoints ({len(joints)}):")
        for joint in joints:
            print(f"  - {joint['name']} ({joint['type']})")
            print(f"    Parent: {joint['parent']} → Child: {joint['child']}")


def main():
    """Main function."""
    urdf_file = '02-humanoid-example.urdf'

    try:
        parser = URDFParser(urdf_file)
        parser.print_structure()
    except FileNotFoundError:
        print(f"Error: URDF file '{urdf_file}' not found.")
    except ET.ParseError as e:
        print(f"Error parsing URDF: {e}")


if __name__ == '__main__':
    main()
