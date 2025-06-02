# Vehicle Modelling and Behaviour Assessment

![Vehicle Dynamics Model](stm.png)

This project simulates the **Single Track (Bicycle) Vehicle Model** using two tyre models—**Linear** and **Non-linear (Pacejka)**—to study vehicle response characteristics: yaw rate, lateral acceleration, body slip angle, and lateral tyre forces under a steering maneuver.

---

## 🚗 Theory

### 🔹 Single Track Model (STM)

The STM simplifies a four-wheeled vehicle into a two-wheel model—one at the front and one at the rear—aligned along the longitudinal axis. It captures essential **lateral dynamics** using two key state variables:
- **Lateral velocity (`v`)**
- **Yaw rate (`r`)**

---

### 🔸 Linear Tire Model

The linear model assumes tire lateral force is directly proportional to slip angle:
\[
F_y = C_\alpha \cdot \alpha
\]
Where:
- \( F_y \): Lateral force
- \( C_\alpha \): Cornering stiffness
- \( \alpha \): Slip angle

✅ Simple and computationally efficient  
❌ Valid only for small slip angles and moderate forces

---

### 🔸 Non-linear Tire Model (Pacejka "Magic Formula")

This model captures tire behavior more realistically:
\[
F_y = D \cdot \sin \left( C \cdot \arctan \left[ B \cdot \alpha - E(B\alpha - \arctan(B\alpha)) \right] \right)
\]
Where:
- \( B, C, D, E \): Empirical curve-fitting parameters
- \( \alpha \): Slip angle

✅ Captures saturation, peak force, and load sensitivity  
❌ More complex and computationally heavier

---

## 🧪 Simulation Setup

- **Vehicle speed:** 80 km/h  
- **Input:** 4° left ramp steering  
- **Simulation duration:** 10 seconds  
- **Solvers used:** `solve_ivp` with Radau method  
- **Models evaluated:** Linear vs. Non-linear

---

## 📈 Results Interpretation

![Simulation Results](plots.gif)

This animation compares key vehicle dynamic responses between the linear and non-linear models:

1. **Yaw Rate**  
   - Non-linear model saturates earlier, showing realistic limitations in vehicle rotation.
2. **Body Slip Angle (β)**  
   - Non-linear model reflects higher slip under load, unlike the conservative linear model.
3. **Lateral Acceleration**  
   - Linear response grows linearly, whereas non-linear saturates reflecting real-world behavior.
4. **Lateral Forces (Front & Rear)**  
   - Linear model oversimplifies; the non-linear model shows asymmetric and nonlinear force behavior.

---

## 📁 Project Structure

```plaintext
📦 Vehicle Modelling Project
├── vehicle_modelling.py        # Main simulation script
├── plots/
│   ├── lateral_acceleration.png      # Lateral acceleration vs time
│   ├── yaw_rate.png                  # Yaw rate vs time
│   ├── body_slip_angle.png          # Body slip angle vs time
│   └── lateral_forces.png           # Tyre lateral forces vs time
├── stm.png                    # Vehicle model diagram
├── plots.gif                  # Simulation output animation
└── README.md                  # Project description and documentation
```
---

## 📌 Conclusion

This project demonstrates the importance of accurate tire modeling in vehicle dynamics simulations. While the linear model is suitable for small-angle, low-demand maneuvers, the non-linear Pacejka model provides crucial fidelity under aggressive steering or high-speed conditions.

---

## 📄 License
This project is open source and available under the MIT License.

### 🙋‍♂️ Author
Anurakt Raj Mathur
💼 [LinkedIn](https://www.linkedin.com/in/anurakt-raj-mathur)
📬 Email: anuraktrajmathur@gmail.com
