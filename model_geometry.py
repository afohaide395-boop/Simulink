"""信号线几何工具：按已序列化端口排布检查，不能替代Simulink真实端口校验。"""
import ast
from collections import defaultdict


def field(element, name, default=None):
    value = element.find(f"P[@Name='{name}']")
    return value.text if value is not None else default


class Geometry:
    def __init__(self, system):
        self.system = system
        self.blocks = {item.get('SID'): item for item in system.findall('Block')}
        self.inputs = defaultdict(int)
        self.outputs = defaultdict(int)
        for wire in system.findall('Line'):
            if wire.get('LineType') == 'Connection':
                continue
            source = field(wire, 'Src')
            if source:
                sid, port = source.split('#out:')
                self.outputs[sid] = max(self.outputs[sid], int(port))
            for destination in wire.findall(".//P[@Name='Dst']"):
                sid, port = destination.text.split('#in:')
                self.inputs[sid] = max(self.inputs[sid], int(port))

    def position(self, sid):
        return ast.literal_eval(field(self.blocks[sid], 'Position'))

    def endpoint(self, endpoint):
        sid, direction = endpoint.split('#')
        side, index = direction.split(':')
        item = self.blocks[sid]
        left, top, right, bottom = self.position(sid)
        counts = item.find('PortCounts')
        inferred = (self.outputs if side == 'out' else self.inputs)[sid]
        count = max(int(counts.get(side, '0')) if counts is not None else 0, inferred, 1)
        mirrored = field(item, 'BlockMirror') == 'on'
        x = right if (side == 'out') != mirrored else left
        y = top + (int(index) - 0.5) * (bottom - top) / count
        # The gate input on the SPS bridge icon is offset, rather than vertically centred.
        if sid == '7' and side == 'in':
            y = top + 9
        return float(x), float(y)

    def polylines(self, wire):
        source = field(wire, 'Src')
        start = self.endpoint(source)

        def walk(node, origin, prefix):
            points = list(prefix)
            point = origin
            encoded = field(node, 'Points')
            if encoded:
                for pair in encoded.strip('[]').split(';'):
                    dx, dy = map(float, pair.split(','))
                    point = point[0] + dx, point[1] + dy
                    points.append(point)
            destination = field(node, 'Dst')
            if destination:
                yield source, destination, points + [self.endpoint(destination)]
            for branch in node.findall('Branch'):
                yield from walk(branch, point, points)

        yield from walk(wire, start, [start])


def is_orthogonal(points, tolerance=1e-8):
    return all(abs(a[0] - b[0]) <= tolerance or abs(a[1] - b[1]) <= tolerance
               for a, b in zip(points, points[1:]))


def crosses_rectangle(a, b, rect, margin=1):
    left, top, right, bottom = rect
    if abs(a[1] - b[1]) < 1e-8:
        return (top + margin < a[1] < bottom - margin and
                max(min(a[0], b[0]), left + margin) < min(max(a[0], b[0]), right - margin))
    if abs(a[0] - b[0]) < 1e-8:
        return (left + margin < a[0] < right - margin and
                max(min(a[1], b[1]), top + margin) < min(max(a[1], b[1]), bottom - margin))
    raise AssertionError('斜线未先被拒绝')


def encode_points(points):
    increments = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:-1])]
    # zip above starts at points[0] and includes each intermediate vertex, excluding the endpoint.
    return '[' + '; '.join(f'{dx:g}, {dy:g}' for dx, dy in increments) + ']'
