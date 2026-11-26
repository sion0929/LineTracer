"""DEV ONLY: make bench `-p name:=value` overrides actually win over the params file.

ROS 2 applies node-specific entries of a params file (`follower_node: ros__parameters: ...`)
after global `-p name:=value` overrides, so a plain `-p perception:=depth` is silently ignored
for any key that rule.yaml sets under follower_node. Scoping the override to the node
(`-p follower_node:name:=value`) makes it take effect.
"""
NODE = 'follower_node'


def node_scoped(args, node=NODE):
    out, i = [], 0
    while i < len(args):
        a = args[i]
        if a in ('-p', '--param') and i + 1 < len(args):
            name, sep, value = args[i + 1].partition(':=')
            if sep and ':' not in name:
                out += [a, f'{node}:{name}:={value}']
            else:
                out += [a, args[i + 1]]
            i += 2
        else:
            out.append(a)
            i += 1
    return out
