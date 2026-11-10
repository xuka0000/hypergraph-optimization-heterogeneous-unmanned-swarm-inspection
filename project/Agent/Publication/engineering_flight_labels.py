"""Exact one-hot flight-membership coupling without a cubic row expansion.

With binary x[i,l] and sum_l x[i,l]=a[i] in {0,1},
R[i]=sum_l (l+1)x[i,l] is an injective encoding of the entire row:
0 means unassigned; 1..n identifies its flight. Thus h[i,j]=1 implies
R[i]=R[j] iff it implies x[i,l]=x[j,l] for every l. This does not
freeze assignment, remove an arc, or change the projected feasible domain.
"""

def add_labels(j):
    return {i:j.aux(sum((l+1)*j.x[i,l] for l in range(j.n)),
                    f'flight_membership_label_{i}',0,j.n,'membership_auxiliary')
            for i in range(j.n)}

def couple(j,labels,i,k):
    j.gate(labels[i]-labels[k],j.h[i,k],0,0,j.n,
           f'same_flight_label_{i}_{k}','assign')

def row_counts(n):
    return dict(old_cubic_rows=2*n*n*(n-1),new_label_rows=n+2*n*(n-1),
                extra_continuous_variables=n,assignment_binary_variables=n*n,
                successor_binary_variables=n*(n-1),
                source='engineering_unified_inspection.JointModel.schedule; gate affine equality emits two inequalities')
