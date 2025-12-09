# Attitude PID tuning notes (safe, step-by-step)

1. Keep props OFF for initial tuning. Use serial prints to observe roll/pitch responses.
2. Start with Ki = 0.0, Kd = 0.0. Increase Kp until you see quick corrective action but no oscillation.
3. Add small Kd to damp oscillations (helps when oscillatory).
4. Add small Ki only if persistent steady-state offset remains.
5. Once bench tuning looks reasonable, move to tethered low-thrust test (very low throttle, props on, with adult).
6. Gradually increase throttle and re-tune GAINs. Always be ready to DISARM.
