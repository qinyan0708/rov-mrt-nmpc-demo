# Mathematical model

## State and input

The planar state and control are

\[
x=[X_M,\;Y_M,\;\psi_M,\;v_M,\;\delta_R]^\mathsf{T},\qquad
u=[a_M,\;\omega_\delta]^\mathsf{T}.
\]

Here \((X_M,Y_M)\) is the MRT reference point, \(\psi_M\) is its heading,
\(v_M\) is forward speed, and \(\delta_R\) is the relative ROV articulation
angle. The inputs are longitudinal acceleration and articulation-angle rate.

## Continuous kinematics

\[
\dot X_M=v_M\cos\psi_M,\quad
\dot Y_M=v_M\sin\psi_M,
\]

\[
\dot\psi_M=\frac{v_M}{L}\tan\delta_R,\quad
\dot v_M=a_M,\quad
\dot\delta_R=\omega_\delta.
\]

The simulator and NMPC both discretize this model with fixed-step fourth-order
Runge-Kutta integration at \(T_s=0.30\) s.

## Road and progress state

The default road is

\[
y_r(x)=1.2\sin\left(\frac{2\pi x}{24}\right),\qquad 0\le x\le48\text{ m}.
\]

It is sampled and reparameterized by arc length \(s\). NMPC optimizes a virtual
progress speed \(v_s\) and propagates

\[
s_{k+1}=\min(s_{\max},s_k+T_s v_{s,k}).
\]

## Articulated geometry

Let \(d(\psi)=[\cos\psi,\sin\psi]^\mathsf{T}\). The hinge and ROV reference
center are

\[
p_H=p_M+L d(\psi_M),\qquad
p_R=p_H+l_R d(\psi_M+\delta_R).
\]

MRT circle centers are \(p_M+\rho_{B,i}d(\psi_M)\), with
\(\rho_B=[-0.90,0.55,2.00]\) m. ROV circle centers are
\(p_R+\rho_{R,i}d(\psi_M+\delta_R)\), with
\(\rho_R=[-0.45,0.45]\) m.

For circle center \(c_i\), predicted obstacle center \(o_{j,k}\), body radius
\(r_i\), obstacle radius \(r_j\), padding \(d_s\), and slack \(\epsilon_k\),
the padded constraint is

\[
\lVert c_i-o_{j,k}\rVert_2^2-(r_i+r_j+d_s)^2+\epsilon_k\ge0.
\]

A second non-slackened constraint uses the smaller hard margin \(d_h\):

\[
\lVert c_i-o_{j,k}\rVert_2^2-(r_i+r_j+d_h)^2\ge0.
\]

Thus slack can relax the preferred clearance but cannot relax the hard
clearance used by this model.
