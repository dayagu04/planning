import json
import textwrap
import networkx as nx
import matplotlib.pyplot as plt

def read_json(file):
    fd = open(file, 'r')
    content = fd.read()
    topics = json.loads(content)
    fd.close()
    return topics

def create_graph(components):
    GG=nx.DiGraph()
    pos = {}
    edges=[]
    colors=[]
    for component in components:
        label = textwrap.fill(component['component_name'], width=50) + '\n'
        label += '-----------------------\n'
        writer_topics = component['writer_topics']
        pos[component['component_name']] = (component['position'])
        colors.append(component['color'])
        for writer_topic in writer_topics:
            label = label + textwrap.fill(writer_topic, width=50) + '\n'
            for component_ in components:
                reader_topics = component_['reader_topics']
                for reader_topic in reader_topics:
                    if writer_topic == reader_topic:
                        edges.append((component['component_name'], component_['component_name']))
        GG.add_node(component['component_name'], desc=label)
    GG.add_edges_from(edges)
    labels = nx.get_node_attributes(GG, 'desc')
    nx.draw(GG, pos, with_labels=True, font_size=3, node_size=2000, node_shape='s', labels=labels, node_color=colors)

topics = read_json('topic.json')
version = topics['version']
components = topics['components']

create_graph(components)
plt.savefig("sys_arch_graph",dpi=1000,bbox_inches = 'tight')
