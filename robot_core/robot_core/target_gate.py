"""Confirm a spatially consistent class across consecutive fresh frames."""
import math


class TargetGate:
    def __init__(self, frames=11, confidence=0.7, max_shift=40.):
        self.frames, self.confidence, self.max_shift = frames, confidence, max_shift
        self.last_stamp_ns = None
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def update(self, boxes, stamp_ns=None):
        if stamp_ns is not None:
            if type(stamp_ns) is not int or stamp_ns <= 0:
                return None
            if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
                return None
            self.last_stamp_ns = stamp_ns
        candidates = [b for b in boxes if b.confidence > self.confidence
                      and b.xmax > b.xmin and b.ymax > b.ymin]
        if not candidates:
            self.reset()
            return None
        box = max(candidates, key=lambda b: b.confidence)
        current = (box.class_name, (box.xmin + box.xmax)/2, (box.ymin + box.ymax)/2)
        same = (self.previous is not None and self.previous[0] == current[0]
                and math.hypot(current[1]-self.previous[1], current[2]-self.previous[2]) <= self.max_shift)
        self.count = self.count + 1 if same else 1
        self.previous = current
        if self.count >= self.frames:
            self.reset()
            return box
        return None
