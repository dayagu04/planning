# Advanced Control Library
## 1. Statespace_sys
### SISO statespace system
Linear SISO statespace system is up to 6th order, including transformation between continuous and discrete system, as well as statespace and transfer function.
### MIMO statespace system
Linear MIMO statespace system with unlimited order, including transformation between continuous and discrete system.

## 2. Filters
### Butterworth filters
Standard lowpass Butterworth filters with poles but no zeros, and low delay Butterworth filters with both zeros and poles.
### Slope filters
Slope filters with bilateral limit rate.
### Differentiators
Proper differentiators with Butterworth coefficients poles, up to 6th order.
### ChebyII filters (On the way)
### Highpass, Bandpass, and Bandstop filters (On the way)

## 3. Compensators
### Basic Loopshaping compensators
Ideal PI-leadlag (boost) system, such as
$$C_{PI-leadlag}(s) = Kp(1 + \frac{2\pi f_{i}}{s})(\frac{\frac{s}{2\pi fc} + 1}{\frac{s}{2\pi fc\cdot  K_{lead-lag}} + 1})$$
and other compensators.
### Notch compensator
Ideal notch compensator such as
$$G_{notch}(s)=\frac{(\frac{s}{2\pi fc})^2 + 2b_{w}(\frac{s}{2\pi fc}) + 1}{(\frac{s}{2\pi fc})^2 + 2\frac{b_{w}}{A}(\frac{s}{2\pi fc}) + 1}$$
## 4. Observers
### luenburgur DOB (Disturbance Observer) with Ideal model
Luenburgur DOB considering ideal model, such as
$$x(s) = \frac{1}{s} u(s)$$
### luenburgur DOB with dynamic model
Luenburgur DOB considering dynamic model, such as
$$ x(s) = \frac{1}{s} (sG(s))u(s) = \frac{1}{s}u_{1}(s) $$
$$u_{1}(s) = (sG(s))u(s)$$
$$ G(s) = \frac{x(s)}{u(s)} $$
$$ \dot{x} = sG(s)u + d = u_{1} + d $$

$$ \dot{\hat{x}} = u_{1} + \hat{d} + k_{1}(x - \hat{x}) $$

$$\dot{\hat{d}} = k_{2}(x - \hat({x})) $$

## 5. Vehicle dynamics
### Kinematics considering longitude acc response model and latitude delta response model
$$G_{acc}(s) = \frac{1}{\tau _{acc}s + 1} $$

$$G_{\delta}(s) = \frac{1}{\tau _{\delta}s + 1}$$
