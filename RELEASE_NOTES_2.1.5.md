# [2.1.5] – 2025-10-27

This release introduces unlockable factory anchors, allowing users to reverse anchor locks when recalibration is necessary. It also includes significant improvements to I2C communication and ADC configuration for Gen 4 boards, along with enhanced protection for factory calibration data.

⚠️ **This firmware requires GUI version 2.1.3 or later** for full compatibility with the new unlock anchors feature.

## ✨ Added
**Unlockable Factory Anchors**: Anchor locks are now reversible. Users can unlock previously locked anchors if recalibration is needed, while maintaining protection against accidental modification.

## 🐛 Fixed
**I2C Communication (Gen 4 Boards)**: Improved I2C stability and reliability for Gen 4 control boards, resolving communication issues with external ADCs.

**ADS1115 ADC Configuration**: Updated PGA (Programmable Gain Amplifier) settings for improved analog input accuracy and reliability on Gen 4 boards.

## 🔁 Modified
**Force Anchor Protection**: Strengthened overwrite protection to better safeguard factory-programmed calibration data during device configuration updates.

**Memory Layout**: Adjusted force anchor and configuration storage addresses to improve alignment with protected flash regions.

**I2C Pin Configuration**: Optimized I2C pin assignments (PB10/PB11) for Gen 4 board compatibility.

## ⚙️ Summary
- Added reversible anchor locking for improved flexibility
- Enhanced I2C communication reliability for Gen 4 hardware
- Improved ADC accuracy and configuration
- Strengthened protection for factory calibration data
