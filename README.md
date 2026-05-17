## Figure ve Kod Grafik İsimleri

- Figure 8 → Step steer path
- Figure 9 → Step steer results
- Figure 10 → Circuit results
- Figure 11 → Vehicle test crosstrack error
- Figure 12 → Zoomed path
- Figure 4 benzeri grafik → Road data

## Makalede ve Kodda Kullanılan Ortak Parametreler

- Wheelbase: L = 2.07 m
- Front axle distance: a = 0.91 m
- Rear axle distance: b = 1.16 m
- Vehicle mass: m = 394.4 kg
- Front cornering stiffness: Cyf = 28000 N/rad
- Rear cornering stiffness: Cyr = 26000 N/rad
- Step-steer radius: R = 12 m
- Lateral offset: 0.5 m
- Test speeds: 3 m/s and 8 m/s
- Steering limit: deltaMax = 35 deg

## Denklemler

- Look-ahead distance:  
  s_look = s + v*tForward

- Heading error:  
  theta_e = psi_ref - psi

- Curvature feedforward:  
  delta_ff = atan(L*kappa)

- Stanley control law:  
  delta_cmd = delta_ff + theta_e + atan(k*e/(kSoft+v)) + kd*(r_ref-r)

- Reference yaw rate:  
  r_ref = v*kappa

- Crosstrack error:  
  e = -(x_ref-x)*sin(psi_ref) + (y_ref-y)*cos(psi_ref)

- Front slip angle:  
  alpha_f = delta - beta - a*r/v

- Rear slip angle:  
  alpha_r = -beta + b*r/v

- Tire lateral forces:  
  F_yf = Cyf*alpha_f  
  F_yr = Cyr*alpha_r

- Dynamic bicycle model:  
  beta_dot = (F_yf + F_yr)/(m*v) - r  
  r_dot = (a*F_yf - b*F_yr)/Iz

- Vehicle motion:  
  x_dot = v*cos(psi+beta)  
  y_dot = v*sin(psi+beta)  
  psi_dot = r
