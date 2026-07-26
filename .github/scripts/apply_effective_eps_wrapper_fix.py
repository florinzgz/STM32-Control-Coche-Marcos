from pathlib import Path

path = Path("Core/Src/motor_control_patched.c")
text = path.read_text(encoding="utf-8")


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    text = text.replace(old, new, 1)


replace_once(
    """    if (!steering_calibrated) {
        Steering_Neutralize();
        return;
    }
    if (enc_fault) {
""",
    """    if (!steering_calibrated) {
        Steering_Neutralize();
        return;
    }

    /* The Makefile compiles this wrapper, not motor_control.c directly.  Keep
     * the effective EPS writer electrically gated by the completed relay
     * sequence and by the commanded state of the steering power relay. */
    if (!Safety_IsPowerReady() ||
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) != GPIO_PIN_SET) {
        Steering_Neutralize();
        return;
    }

    if (enc_fault) {
""",
    "effective EPS power gate",
)

replace_once(
    """    float abs_pct = fabsf(pwm_pct);
    if (abs_pct > 0.01f && abs_pct < p->min_drive_pct) {
        pwm_pct = (pwm_pct > 0.0f) ? p->min_drive_pct : -p->min_drive_pct;
        abs_pct = p->min_drive_pct;
    }

    if (abs_pct < p->coast_band_pct) {
        Steering_Neutralize();
        return;
    }
""",
    """    /* Resolve COAST before minimum-drive compensation.  A tiny control
     * residue must not be promoted to the configured breakaway PWM and keep
     * the BTS7960 fighting the driver near centre. */
    const EpsOutputDecision_t output = EpsOutput_Resolve(
        pwm_pct, p->coast_band_pct, p->min_drive_pct);
    if (output.coast) {
        Steering_Neutralize();
        return;
    }
    pwm_pct = output.pwm_pct;
""",
    "effective EPS coast-first policy",
)

path.write_text(text, encoding="utf-8")
