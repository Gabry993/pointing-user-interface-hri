from collections import defaultdict
from typing import Any, Dict, List, Optional, Type, Tuple, Mapping, Iterable
import math
from functools import total_ordering
import numpy as np

import yaml
import shapely.geometry
import shapely.ops

import conveyor_msgs.msg

Range = Tuple[float, float]
Position = Tuple[float, float, float]
LED = Tuple[str, Position]


class Belt:
    def __init__(self, uid: str, centerline: shapely.geometry.LineString, width: float):
        self.uid = uid
        self.centerline = centerline
        self.width = width

    def __hash__(self) -> int:
        return hash(self.uid)

    def __repr__(self) -> str:
        return repr(self.uid)

    @property
    def length(self) -> float:
        return self.centerline.length

    @classmethod
    def load(cls: Type, uid: int, centerline: str, width: float, **kwargs: Any) -> 'Belt':
        return cls(uid=uid, width=width, centerline=shapely.geometry.LineString(centerline))

    @classmethod
    def from_strip(cls: Type, strip: 'Strip', width: float = 0.1) -> 'Belt':
        return cls(uid=f'strip_{strip.uid}', width=width, centerline=strip.line)


@total_ordering
class Strip:
    def __init__(self, uid: int, line: shapely.geometry.LineString, pixels: int, direction: int):
        self.uid = uid
        self.line = line
        self.pixels = pixels
        self.direction = direction

    @property
    def length(self) -> float:
        return self.line.length

    def draw(self, color: List[int], intervals: List[Range]) -> np.ndarray:
        data: List[List[int]] = [[0, 0, 0]] * (self.pixels)
        le = self.pixels
        for a, b in intervals:
            i0 = math.ceil(a * le)
            i1 = math.floor(b * le)
            data[i0:i1] = [color] * (i1 - i0)
        if self.direction == -1:
            data = data[::-1]
        return np.array(data)

    def __hash__(self) -> int:
        return hash(self.uid)

    def __repr__(self) -> str:
        return repr(self.uid)

    def __eq__(self, other: Any) -> bool:
        return self is other

    def __ne__(self, other: Any) -> bool:
        return not (self == other)

    def __lt__(self, other: 'Strip') -> bool:
        return self.uid < other.uid

    @classmethod
    def load(cls, uid: int, line: str, pixels: int, direction: int, **kwargs: Any) -> 'Strip':
        return cls(uid=uid, pixels=pixels, line=shapely.geometry.LineString(line),
                   direction=direction)


DecomposedBelt = List[Tuple[Range, Tuple[Strip, Range]]]
Decomposition = Mapping[Belt, DecomposedBelt]
PositionOnBelt = Tuple[Belt, float]
IntervalOnStrip = Tuple[Strip, Range]


def overlap(dec: DecomposedBelt, a: float, b: float, belt: Belt) -> DecomposedBelt:
    rs: DecomposedBelt = []
    for ((c, d), strip_interval) in dec:
        if b < c or a > d:
            continue
        strip, (e, f) = strip_interval
        x, y = c, d
        if b < d:
            y = b
            p = belt.centerline.interpolate(b, normalized=True)
            f = strip.line.project(p, normalized=True)
        if a > c:
            x = a
            p = belt.centerline.interpolate(a, normalized=True)
            e = strip.line.project(p, normalized=True)
        rs.append(((x, y), (strip, (e, f))))
    return rs


class LEDs:

    def __init__(self, leds: List[LED]) -> None:
        self.leds = {led[0]: led[1] for led in leds}

    @classmethod
    def load(cls, data: Dict[str, Any], **kwargs: Any) -> 'LEDs':
        return cls(
            leds=[(uid, value['position']) for uid, value in data.get('leds', {}).items()])

    @classmethod
    def load_file(cls, path: str, **kwargs: Any) -> 'LEDs':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            return cls.load(data, **kwargs)


class Map:

    @property
    def belts(self) -> Iterable[Belt]:
        return self._belts.values()

    @property
    def bounding_box(self) -> Tuple[float, float, float, float]:
        union = shapely.ops.unary_union([belt.centerline for belt in self.belts])
        return union.bounds

    @property
    def strips(self) -> Iterable[Strip]:
        return self._strips.values()

    def point_from_msg(self, msg: conveyor_msgs.msg.PositionOnStrip) -> Tuple[float, float, float]:
        belt = self._belts[msg.name]
        x, y, z = belt.centerline.interpolate(msg.position, normalized=True).coords[0]
        return x, y, z

    def __init__(self, belts: List[Belt], strips: List[Strip], links: List[Tuple[str, str]],
                 tol: float = 0.01, tol_o: float = 1e-3) -> None:
        if not belts:
            # There are only strips ... emulate belts with strips
            belts = [Belt.from_strip(strip) for strip in strips]
        self._belts = {b.uid: b for b in belts}
        self._strips = {s.uid: s for s in strips}

        self._next_belt: Dict[Belt, Belt] = {}
        self._previous_belt: Dict[Belt, Belt] = {}
        for b1 in self.belts:
            p1 = b1.centerline.interpolate(0, normalized=True)
            p2 = b1.centerline.interpolate(1, normalized=True)
            m = b1.width * 0.5 + tol
            for b2 in self.belts:
                if b1 is b2:
                    continue
                if p1.distance(b2.centerline) < m:
                    self._previous_belt[b1] = b2
                if p2.distance(b2.centerline) < m:
                    self._next_belt[b1] = b2
        if links:
            for b1_id, b2_id in links:
                b1, b2 = self._belts[b1_id], self._belts[b2_id]
                self._next_belt[b1] = b2
                self._previous_belt[b2] = b1

        dec: Decomposition = defaultdict(list)
        for strip in self.strips:
            for belt in self.belts:
                l1 = strip.line
                z1 = l1.coords[0][2]
                l2 = belt.centerline
                z2 = l2.coords[0][2]
                d = l1.distance(l2)
                if (abs(z1 - z2) < 0.5 and d < 0.5 * belt.width + tol):
                    a = l1.project(l2.interpolate(0, normalized=True), normalized=True)
                    b = l1.project(l2.interpolate(1, normalized=True), normalized=True)
                    c = l2.project(l1.interpolate(a, normalized=True), normalized=True)
                    d = l2.project(l1.interpolate(b, normalized=True), normalized=True)
                    if (b - a) > tol_o and (d - c) > tol_o:
                        dec[belt].append(((c, d), (strip, (a, b))))
        self.dec = dec

    def draw(self) -> None:
        from matplotlib import pyplot
        for belt in self.belts:
            pyplot.plot(*belt.centerline.xy, '-', label=f'Belt {belt}')
        for strip in self.strips:
            pyplot.plot(*strip.line.xy, '--', label=f'Strip {strip}')
        pyplot.legend()
        pyplot.axis('equal')

    @classmethod
    def load(cls, data: Dict[str, Any], **kwargs: Any) -> 'Map':
        return cls(
            belts=[Belt.load(uid=uid, **value) for uid, value in data.get('belts', {}).items()],
            strips=[Strip.load(uid=uid, **value) for uid, value in data.get('strips', {}).items()],
            links=[(link['from'], link['to']) for link in data.get('links', [])], **kwargs)

    @classmethod
    def load_file(cls, path: str, **kwargs: Any) -> 'Map':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            return cls.load(data, **kwargs)

    def project_interval(self, a: PositionOnBelt, b: PositionOnBelt) -> Decomposition:
        (belt_a, s_a) = a
        (belt_b, s_b) = b
        if belt_a is belt_b:
            return {belt_a: overlap(self.dec[belt_a], s_a, s_b, belt_a)}
        return {belt_a: overlap(self.dec[belt_a], s_a, 1, belt_a),
                belt_b: overlap(self.dec[belt_b], 0, s_b, belt_b)}

    def next_belt(self, belt: Belt) -> Optional[Belt]:
        return self._next_belt.get(belt)

    def previous_belt(self, belt: Belt) -> Optional[Belt]:
        return self._previous_belt.get(belt)

    def interval(self, belt: Belt, position: float, width: float
                 ) -> Tuple[PositionOnBelt, PositionOnBelt]:
        le = belt.centerline.length
        s = position * belt.centerline.length
        a = s - width / 2
        b = s + width / 2
        p1, p2 = (belt, a / le), (belt, b / le)
        if a < 0:
            p_belt = self.previous_belt(belt)
            if p_belt:
                a += p_belt.centerline.length
                p1 = (p_belt, a / p_belt.centerline.length)
            else:
                p1 = (belt, 0)
        else:
            p1 = (belt, a / le)
        if b > le:
            n_belt = self.next_belt(belt)
            if n_belt:
                b -= belt.centerline.length
                p2 = (n_belt, b / n_belt.centerline.length)
            else:
                p2 = (belt, 1)
        else:
            p2 = (belt, b / le)
        return p1, p2

    def strips_near(self, belt_uid: str, position: float, width: float
                    ) -> List[IntervalOnStrip]:
        belt = self._belts[belt_uid]
        a, b = self.interval(belt, position, width)
        projection = self.project_interval(a, b)
        return [strip_interval for intervals in projection.values()
                for _, strip_interval in intervals if strip_interval]
