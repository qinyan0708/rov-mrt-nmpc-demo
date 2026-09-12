# Controller design

The controller follows a model-predictive contouring-control structure. Each
optimization stage penalizes contour error, lag error, heading error, speed
error, input magnitude, and input change, while rewarding forward progress.

For the demonstrated two-obstacle case, the contour target is shifted by a
smooth prescribed profile. The first cosine segment moves the target to the
positive side of the road, a middle cosine segment transitions to the negative
side, and the last segment returns it to zero. This profile is scenario logic,
not an online passing-side optimizer.

At each 0.30 s callback the node:

1. receives the five-state vehicle estimate and two obstacle records;
2. propagates constant-velocity obstacle predictions over 14 steps;
3. solves the constrained nonlinear program with CasADi/IPOPT;
4. applies the first acceleration and steering-rate input;
5. shifts the solution to warm-start the next solve;
6. publishes predicted motion and safety/timing diagnostics.

The preferred padded constraints share one nonnegative slack per stage,
penalized by \(10^5\epsilon_k^2\). Hard-clearance constraints have no slack.

If a solution is rejected, the controller commands bounded deceleration and a
bounded steering-centering rate. This is a practical simulation fallback, not
a formally verified backup controller. The vehicle node independently applies
the same braking behavior if commands become stale.
