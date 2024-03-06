'''
codes are from https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
'''

class InArena:
    def __init__(self, arena_edges):
        self.hull = arena_edges
    def is_divide_pt(self,x11, y11, x12, y12, x21, y21, x22, y22):
        # 직선의 양분 판단
        f1 = (x12 - x11) * (y21 - y11) - (y12 - y11) * (x21 - x11)
        f2 = (x12 - x11) * (y22 - y11) - (y12 - y11) * (x22 - x11)
        if f1 * f2 < 0:
            return True
        else:
            return False
    def is_cross_pt(self,x11,y11, x12,y12, x21,y21, x22,y22):
        b1 = self.is_divide_pt(x11,y11, x12,y12, x21,y21, x22,y22)
        b2 = self.is_divide_pt(x21,y21, x22,y22, x11,y11, x12,y12)
        if b1 and b2:
            return True
        else:
            return False
    def is_in_arena(self, point):
        cnt = 0
        for i in range(len(self.hull)):
            p1 = self.hull[i]
            p2 = self.hull[(i+1) % len(self.hull)]
            if self.is_cross_pt(p1[0], p1[1], p2[0], p2[1], point[0], point[1], point[0]+99999, point[1]+99999):
                cnt += 1
        return cnt % 2 == 1
