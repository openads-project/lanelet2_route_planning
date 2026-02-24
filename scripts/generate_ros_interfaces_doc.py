#!/usr/bin/env python3
"""
Auto-generates ROS2 interface documentation (topics, actions, parameters)
by parsing C++ source files in a ROS2 repository.

Usage:
    python3 scripts/generate_ros_interfaces_doc.py [REPO_ROOT]

REPO_ROOT defaults to the script's parent directory.
Output is printed to stdout in Markdown format.
"""

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class TopicInterface:
    name: str
    msg_type: str


@dataclass
class ActionInterface:
    name: str
    action_type: str


@dataclass
class Parameter:
    name: str
    ros_type: str
    default: str
    description: str


@dataclass
class NodeInterfaces:
    node_name: str
    subscribers: list = field(default_factory=list)
    publishers: list = field(default_factory=list)
    action_servers: list = field(default_factory=list)
    action_clients: list = field(default_factory=list)
    parameters: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Type helpers
# ---------------------------------------------------------------------------

def cpp_ros_type(cpp_type: str, type_aliases: dict) -> str:
    """Resolve type aliases and convert C++ ROS type to ROS notation (:: -> /)."""
    t = cpp_type.strip()
    t = type_aliases.get(t, t)
    return t.replace('::', '/').strip()


def cpp_param_type(cpp_type: str) -> str:
    """Map C++ type to ROS parameter type name."""
    t = cpp_type.strip()
    if t in ('double', 'float'):
        return 'float'
    if t == 'int':
        return 'int'
    if t == 'bool':
        return 'bool'
    if t == 'std::string':
        return 'string'
    if 'vector' in t:
        return 'string[]'
    return t


def format_default(default_str: Optional[str], cpp_type: str) -> str:
    """Format a C++ default value for Markdown display."""
    if default_str is None:
        return '[]' if 'vector' in cpp_type else ''
    val = default_str.strip()
    if val.startswith('"') and val.endswith('"'):
        val = val[1:-1]
    return val


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def find_packages(repo_root: Path) -> list:
    """Return [(package_name, package_dir)] for all ROS packages found."""
    packages = []
    for pkg_xml in sorted(repo_root.rglob('package.xml')):
        try:
            root = ElementTree.parse(pkg_xml).getroot()
            name_el = root.find('name')
            if name_el is not None and name_el.text:
                packages.append((name_el.text.strip(), pkg_xml.parent))
        except ElementTree.ParseError:
            pass
    return packages


def find_node_sources(package_dir: Path) -> list:
    """Return .cpp files that define a ROS node (contain ': Node("..." )')."""
    node_pattern = re.compile(r':\s*Node\s*\(')
    return [
        cpp for cpp in sorted(package_dir.rglob('*.cpp'))
        if node_pattern.search(cpp.read_text(errors='replace'))
    ]


def find_headers(package_dir: Path) -> list:
    """Return all header files (.hpp, .h) in the package."""
    return [
        p for ext in ('*.hpp', '*.h')
        for p in sorted(package_dir.rglob(ext))
    ]


# ---------------------------------------------------------------------------
# Extraction from source/header text
# ---------------------------------------------------------------------------

def extract_node_name(source: str) -> Optional[str]:
    m = re.search(r':\s*Node\s*\(\s*"([^"]+)"', source)
    return m.group(1) if m else None


def extract_subscribers(source: str, aliases: dict) -> list:
    return [
        TopicInterface(name=m.group(2), msg_type=cpp_ros_type(m.group(1), aliases))
        for m in re.finditer(r'create_subscription\s*<([^>]+)>\s*\(\s*"([^"]+)"', source)
    ]


def extract_publishers(source: str, aliases: dict) -> list:
    return [
        TopicInterface(name=m.group(2), msg_type=cpp_ros_type(m.group(1), aliases))
        for m in re.finditer(r'create_publisher\s*<([^>]+)>\s*\(\s*"([^"]+)"', source)
    ]


def extract_action_servers(source: str, aliases: dict) -> list:
    return [
        ActionInterface(name=m.group(2), action_type=cpp_ros_type(m.group(1), aliases))
        for m in re.finditer(
            r'rclcpp_action::create_server\s*<([^>]+)>\s*\(\s*this\s*,\s*"([^"]+)"', source)
    ]


def extract_action_clients(source: str, aliases: dict) -> list:
    return [
        ActionInterface(name=m.group(2), action_type=cpp_ros_type(m.group(1), aliases))
        for m in re.finditer(
            r'rclcpp_action::create_client\s*<([^>]+)>\s*\(\s*this\s*,\s*"([^"]+)"', source)
    ]


def extract_raw_parameters(source: str) -> list:
    """Return [(param_name, member_var_name, description)] from declareAndLoadParameter calls."""
    return [
        (m.group(1), m.group(2), m.group(3))
        for m in re.finditer(
            r'declareAndLoadParameter\s*\(\s*"([^"]+)"\s*,\s*(\w+)\s*,\s*"([^"]+)"', source)
    ]


def build_member_var_map(headers: list) -> dict:
    """Return {var_name: (cpp_type, default_str)} from member variable declarations."""
    pattern = re.compile(
        r'\b(double|float|int|bool|std::string|std::vector\s*<[^>]+>)\s+(\w+_)\s*'
        r'(?:=\s*([^;{]+))?;'
    )
    result = {}
    for header in headers:
        for m in pattern.finditer(header.read_text(errors='replace')):
            var_name = m.group(2).strip()
            if var_name not in result:
                result[var_name] = (m.group(1).strip(), m.group(3))
    return result


def build_type_alias_map(headers: list) -> dict:
    """Return {alias: full_cpp_type} from 'using Alias = FullType;' declarations."""
    pattern = re.compile(r'\busing\s+(\w+)\s*=\s*([^;]+);')
    result = {}
    for header in headers:
        for m in pattern.finditer(header.read_text(errors='replace')):
            result[m.group(1).strip()] = m.group(2).strip()
    return result


def resolve_parameters(raw_params: list, member_var_map: dict) -> list:
    params = []
    for name, member_var, description in raw_params:
        cpp_type, default_raw = member_var_map.get(member_var, ('', None))
        params.append(Parameter(
            name=name,
            ros_type=cpp_param_type(cpp_type),
            default=format_default(default_raw, cpp_type),
            description=description,
        ))
    return params


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------

def md_topic_table(interfaces: list) -> str:
    rows = ['| Topic | Type | Description |', '| --- | --- | --- |']
    rows += [f'| `{i.name}` | `{i.msg_type}` | |' for i in interfaces]
    return '\n'.join(rows)


def md_action_table(interfaces: list) -> str:
    rows = ['| Action | Type | Description |', '| --- | --- | --- |']
    rows += [f'| `{i.name}` | `{i.action_type}` | |' for i in interfaces]
    return '\n'.join(rows)


def md_parameter_table(params: list) -> str:
    rows = ['| Parameter | Type | Default | Description |', '| --- | --- | --- | --- |']
    rows += [f'| `{p.name}` | `{p.ros_type}` | `{p.default}` | {p.description} |'
             for p in params]
    return '\n'.join(rows)


def render_node(node: NodeInterfaces) -> str:
    parts = [f'## `{node.node_name}`']
    if node.subscribers:
        parts += ['\n### Subscribed Topics\n', md_topic_table(node.subscribers)]
    if node.publishers:
        parts += ['\n### Published Topics\n', md_topic_table(node.publishers)]
    if node.action_servers:
        parts += ['\n### Actions\n', md_action_table(node.action_servers)]
    if node.action_clients:
        parts += ['\n### Action Clients\n', md_action_table(node.action_clients)]
    if node.parameters:
        parts += ['\n### Parameters\n', md_parameter_table(node.parameters)]
    return '\n'.join(parts)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    repo_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent

    packages = find_packages(repo_root)
    if not packages:
        print('No ROS packages found.', file=sys.stderr)
        sys.exit(1)

    output_blocks = []
    for _pkg_name, pkg_dir in packages:
        node_sources = find_node_sources(pkg_dir)
        if not node_sources:
            continue

        headers = find_headers(pkg_dir)
        member_var_map = build_member_var_map(headers)
        type_aliases = build_type_alias_map(headers)

        for source_file in node_sources:
            source = source_file.read_text(errors='replace')
            node = NodeInterfaces(
                node_name=extract_node_name(source) or source_file.stem,
                subscribers=extract_subscribers(source, type_aliases),
                publishers=extract_publishers(source, type_aliases),
                action_servers=extract_action_servers(source, type_aliases),
                action_clients=extract_action_clients(source, type_aliases),
                parameters=resolve_parameters(extract_raw_parameters(source), member_var_map),
            )
            output_blocks.append(render_node(node))

    print('\n\n'.join(output_blocks))


if __name__ == '__main__':
    main()
