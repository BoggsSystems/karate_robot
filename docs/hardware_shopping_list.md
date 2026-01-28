# Sensei Robot Hardware Shopping List

## Build Specification: Recommended (~$1,000)

**Robot**: 14-DOF Humanoid, ~30cm tall  
**Budget**: ~$1,000  
**Build Time**: 4-6 weeks

---

## Complete Parts List

### 1. Compute - Brain ($80)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| Raspberry Pi 5 (8GB) | 1 | SC1112 | [Adafruit](https://www.adafruit.com/product/5813) / [PiShop](https://www.pishop.us/) | $80 |
| Pi 5 Active Cooler | 1 | SC1148 | Adafruit | $5 |
| 32GB microSD (fast) | 1 | SanDisk Extreme | Amazon | $12 |
| USB-C Power Supply (27W) | 1 | Official Pi 5 PSU | Adafruit | $12 |

**Subtotal: ~$109**

---

### 2. Compute - Spine MCU ($35)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| Teensy 4.1 | 1 | DEV-16771 | [PJRC](https://www.pjrc.com/store/teensy41.html) / SparkFun | $30 |
| Teensy 4.1 Pins | 1 | Included or separate | PJRC | $3 |

**Subtotal: ~$33**

---

### 3. Actuators - Servos ($700)

| Item | Qty | Part Number | Supplier | Price/ea | Total |
|------|-----|-------------|----------|----------|-------|
| Dynamixel XL430-W250-T | 14 | 902-0135-000 | [Robotis](https://www.robotis.us/) / [TrossenRobotics](https://www.trossenrobotics.com/) | $49.90 | $699 |
| U2D2 (USB-Dynamixel interface) | 1 | 902-0132-000 | Robotis | $35 | $35 |
| U2D2 Power Hub | 1 | 902-0145-000 | Robotis | $20 | $20 |
| Dynamixel Cable (200mm) | 10 | 903-0187-000 | Robotis | $2 | $20 |
| Dynamixel Cable (100mm) | 10 | 903-0186-000 | Robotis | $1.50 | $15 |

**Subtotal: ~$789**

#### Alternative: Budget Servo Option (~$350)
| Item | Qty | Part Number | Supplier | Price/ea | Total |
|------|-----|-------------|----------|----------|-------|
| Feetech STS3215 | 14 | STS3215 | [AliExpress](https://aliexpress.com) | $22 | $308 |
| FE-URT-1 (USB interface) | 1 | FE-URT-1 | AliExpress | $15 | $15 |

*Note: Feetech servos have less precise feedback but are 50% cheaper*

---

### 4. Sensors ($45)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| BNO085 IMU Breakout | 1 | 4754 | [Adafruit](https://www.adafruit.com/product/4754) | $20 |
| FSR 402 (Force Sensitive Resistor) | 4 | SEN-09375 | [SparkFun](https://www.sparkfun.com/products/9375) | $7 ea ($28) |
| 10k Resistors (for FSR) | 10 | - | Amazon | $2 |

**Subtotal: ~$50**

---

### 5. Power System ($60)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| 2S LiPo 2200mAh 25C | 2 | - | Amazon/HobbyKing | $20 ea ($40) |
| 2S LiPo Balance Charger | 1 | IMAX B6 or similar | Amazon | $25 |
| XT60 Connectors (pair) | 5 | - | Amazon | $8 |
| 5V 3A Buck Converter | 2 | D24V30F5 | [Pololu](https://www.pololu.com/product/2851) | $7 ea ($14) |
| Power Switch (high current) | 1 | - | Amazon | $5 |
| LiPo Voltage Alarm | 2 | - | Amazon | $5 |

**Subtotal: ~$97**

---

### 6. Frame & Mechanical ($80)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| PETG Filament (1kg) | 2 | - | Amazon/Overture | $22 ea ($44) |
| TPU Filament (for feet) | 1 | - | Amazon | $25 |
| M2 Screw Assortment | 1 | - | Amazon | $12 |
| M2.5 Screw Assortment | 1 | - | Amazon | $10 |
| M3 Screw Assortment | 1 | - | Amazon | $10 |
| Brass Heat-Set Inserts M2/M2.5/M3 | 1 kit | - | Amazon | $15 |

**Subtotal: ~$116**

*Note: Assumes access to a 3D printer. Add ~$200-300 for an Ender 3 V2 if needed.*

---

### 7. Electronics & Wiring ($50)

| Item | Qty | Part Number | Supplier | Price |
|------|-----|-------------|----------|-------|
| Prototype PCB (5x7cm) | 5 | - | Amazon | $8 |
| 22AWG Silicone Wire (assorted) | 1 set | - | Amazon | $15 |
| JST-XH Connector Kit | 1 | - | Amazon | $12 |
| Dupont Connector Kit | 1 | - | Amazon | $10 |
| Heat Shrink Tubing | 1 | - | Amazon | $8 |
| Wire Strippers/Crimpers | 1 | - | Amazon | $15 |

**Subtotal: ~$68**

---

### 8. Optional Upgrades

| Item | Purpose | Price |
|------|---------|-------|
| OAK-D Lite Camera | Vision/depth | $150 |
| E-Stop Button (mushroom) | Safety | $10 |
| LED Ring (status) | Visual feedback | $8 |
| Speaker/Buzzer | Audio feedback | $5 |
| Extra LiPo batteries | Extended runtime | $40 |

---

## Summary

| Category | Cost |
|----------|------|
| Brain (Pi 5 + accessories) | $109 |
| Spine (Teensy 4.1) | $33 |
| Servos (Dynamixel XL430 x14) | $789 |
| Sensors (IMU + foot) | $50 |
| Power System | $97 |
| Frame & Mechanical | $116 |
| Electronics & Wiring | $68 |
| **TOTAL** | **$1,262** |

### Budget Version (Feetech servos): ~$800

---

## Supplier Summary

| Supplier | Items | Est. Total |
|----------|-------|------------|
| **Robotis/Trossen** | Dynamixels, U2D2 | $789 |
| **Adafruit** | Pi 5, BNO085 | $105 |
| **SparkFun** | FSR sensors | $30 |
| **PJRC** | Teensy 4.1 | $33 |
| **Amazon** | Everything else | $300 |

---

## Order Priority

### Week 1: Order First (longest lead time)
- [ ] Dynamixel servos (may ship from Korea)
- [ ] Raspberry Pi 5 (check availability)
- [ ] Teensy 4.1

### Week 2: Order Second
- [ ] Sensors (BNO085, FSR)
- [ ] Power components
- [ ] Wiring/connectors

### Week 3: Order as Needed
- [ ] Filament (if not in stock)
- [ ] Hardware (screws, inserts)

---

## Tools Required

| Tool | Have? | Cost if Needed |
|------|-------|----------------|
| 3D Printer | | $200-300 |
| Soldering Iron | | $30-50 |
| Multimeter | | $20-30 |
| Hex Driver Set | | $15 |
| Wire Strippers | | $15 |
| Crimping Tool | | $20 |

---

## Notes

1. **Dynamixel Servo IDs**: Will need to set unique IDs (1-14) on each servo before assembly
2. **3D Print Time**: Expect 50-80 hours of printing for full frame
3. **Assembly**: Budget 20-30 hours for assembly and wiring
4. **Software**: ROS2 Humble + trained policy ready to deploy

---

*Created: January 28, 2026*
