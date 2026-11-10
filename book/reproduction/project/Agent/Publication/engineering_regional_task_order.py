"""Free initializer orderings across towers, without fixed tower blocks.

Exact shared viewpoints stay together only in these proposals. The original
joint model keeps every predecessor and continuous viewpoint decision free.
"""
import math


def proposals(tasks,observations,start_xy,end_xy):
    groups={}
    for i in tasks:
        key=tuple(float(x) for x in observations[i]['xyz_m'])
        groups.setdefault(key,[]).append(i)
    points=list(groups);N=len(points)
    def cost(a,b):
        return max(math.dist(a[:2],b[:2])/4.5,abs(a[2]-b[2])/.9)
    edge=[[cost(a,b) for b in points] for a in points]
    begin=[math.dist(start_xy,p[:2])/4.5 for p in points]
    end=[math.dist(end_xy,p[:2])/4.5 for p in points]
    def total(order):
        return begin[order[0]]+end[order[-1]]+sum(edge[a][b] for a,b in zip(order,order[1:]))
    routes=[]
    old=[]
    for i in tasks:
        k=points.index(tuple(float(x) for x in observations[i]['xyz_m']))
        if k not in old:old.append(k)
    routes.append(old)
    for first in sorted(range(N),key=lambda i:begin[i])[:min(6,N)]:
        order=[first];left=set(range(N))-{first}
        while left:
            k=min(left,key=lambda j:(edge[order[-1]][j],j));order.append(k);left.remove(k)
        routes.append(order)
    answer={}
    for order in routes:
        while True:
            best=None
            for lo in range(N):
                for hi in range(lo+1,N):
                    oldcost=(begin[order[lo]] if not lo else edge[order[lo-1]][order[lo]])+(end[order[hi]] if hi==N-1 else edge[order[hi]][order[hi+1]])
                    newcost=(begin[order[hi]] if not lo else edge[order[lo-1]][order[hi]])+(end[order[lo]] if hi==N-1 else edge[order[lo]][order[hi+1]])
                    delta=newcost-oldcost
                    if delta < -1e-8 and (best is None or delta<best[0]):best=(delta,lo,hi)
            if best is None:break
            _,lo,hi=best;order=order[:lo]+list(reversed(order[lo:hi+1]))+order[hi+1:]
        for sequence in [order,list(reversed(order))]:
            flat=tuple(i for k in sequence for i in groups[points[k]])
            assert sorted(flat)==sorted(tasks)
            answer[flat]=total(sequence)
    return [list(order) for order,_ in sorted(answer.items(),key=lambda item:item[1])]
