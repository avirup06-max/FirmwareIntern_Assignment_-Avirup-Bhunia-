*MCU:* STM32F411RET6 (LQFP64)  
*Regulator:* AP2112-3.3  
*Clock:* 8 MHz crystal + load caps  
*Debug:* SWD (SWDIO/PA13, SWCLK/PA14, NRST)  
*Interfaces:* UART2 (PA2/PA3), USB FS (PA11/PA12), User LED (PC13), User Button (PA0)

> Keep the bottom layer as a solid GND plane. Use short probes and ESD precautions during bring-up.

---

## 0) Visual & Continuity Checks (before power)
- No shorts between *3V3–GND, **VBUS–GND, **NRST–GND*.
- Crystal and both load caps present and close to *PH0/PH1*.
- *VCAP1 = 2.2 µF to GND, placed right by the pin; **VCAP2* not used on F411.
- One *0.1 µF decoupling cap per VDD pin*, placed within 2–3 mm of each VDD.
- SWD header orientation matches your cable; pin 1 marked.
- If using USB device: *ESD diode* near connector; *22 Ω* series resistors on D+/D−.

---

## 1) Power-Up (no MCU activity yet)
1. *Bench supply or USB* off. Set current limit 200 mA.
2. Power ON. Measure rails:
   - *+5V (VBUS):* 4.75–5.25 V
   - *+3V3:* 3.25–3.35 V (no load dip >150 mV)
3. *AP2112-3.3* checks:
   - EN pin high (if used).
   - Input cap ≥10 µF, output cap 10 µF + local 0.1 µF decouplers.
4. *NRST* pin: ~3.3 V (high). Press reset: dips to <0.3 V then recovers.
5. Current draw at idle (no firmware): typically *< 30–60 mA*.

If anything is off: power down, check orientation, bridges, and shorts.

---

## 2) Clock Verification
- Optional: temporarily route *SYSCLK* via *MCO* to a test pad to verify frequency.
- Alternatively, flash a simple firmware that toggles LED at a known rate; verify timing is correct (indirect HSE check).

Expected: HSE 8 MHz → PLL (e.g., 84 MHz SYSCLK typical) once firmware configures it.

---

## 3) SWD Connectivity
1. Connect *ST-Link: **3V3, GND, SWDIO (PA13), SWCLK (PA14), NRST*.
2. In STM32CubeProgrammer (or OpenOCD), *connect*. You should read device ID and memory map.
3. If connect fails:
   - Hold *NRST* low and try connect under reset.
   - Check SWDIO/SWCLK traces for shorts, wrong orientation, or swapped pins.
   - Verify 3V3 is present at the SWD header (Vref).

---

## 4) Minimal Firmware Sanity
Flash a minimal program:
- Enable HSE + PLL to target SYSCLK (e.g., 84 MHz).
- Toggle *PC13* at 5 Hz (LED proof).
- Read *PA0* with internal pull-up or pull-down (match your schematic).
- Print “Hello” over *USART2* (PA2/PA3) at *9600 8N1*.

*UART loopback test (optional):* Short PA2↔PA3 and ensure sent bytes are received.

---

## 5) USB FS Device (if used)
- Ensure *PA11=D−, PA12=D+* routed with matched lengths; ESD diode close to connector.
- *VBUS sense*: PA9 via divider (e.g., 220 kΩ → PA9 → 100 kΩ → GND). Confirm PA9 < 3.3 V with 5 V on VBUS.
- Load a USB CDC example (or your own). Expect enumeration as a virtual COM port.

If device not detected:
- Check 3.3 V present at MCU.
- Verify *R_USB D+/D−* series resistors are 22 Ω each.
- Verify PA9 senses VBUS high.
- Try a different cable/PC port.

---

## 6) Final Functional Checklist
- [ ] 3V3 rail within spec under load
- [ ] NRST acts correctly, no unintended brownouts
- [ ] ST-Link connects consistently
- [ ] LED blinks on PC13
- [ ] Button PA0 toggles logic in debugger
- [ ] UART2 TX/RX functional at 9600 8N1
- [ ] (Optional) USB CDC enumerates on host PC
- [ ] No excessive heating on LDO or MCU

---

## 7) Troubleshooting Quick Table
| Symptom | Likely Cause | Fix |
|---|---|---|
| No ST-Link connect | SWD pins swapped, NRST held low, no 3V3 at Vref | Verify header mapping, release NRST, check 3V3 |
| Random resets | VCAP1 missing/too far, poor decoupling | Place 2.2 µF at VCAP1, add 0.1 µF near each VDD |
| UART gibberish | Wrong baud or clock tree | Recalculate BRR; ensure PLL/HSE configured |
| USB no enumerate | Missing VBUS sense on PA9, bad D+/D− routing | Add divider to PA9; check 22 Ω resistors; ESD near connector |
| Crystal won’t start | Load caps wrong, long traces | Adjust to CL, keep loop tight, add GND pour around crystal |

---

## 8) Test Points (recommended)
- *TP_VDD, TP_GND, TP_NRST, TP_TX, TP_RX, TP_MCO (optional)*
