//
// Native Instruments Traktor Kontrol Z1 HID controller script for Mixxx 2.4
// -------------------------------------------------------------------------
// Based on: NI Traktor Kontrol series scripts by leifhelm, mi01 & xeruf
// Author: djantti
//

// LED brightness levels
const ledLevels = {
    off: 0x00,
    low: 0x0A,
    medium: 0x1F,
    high: 0x3F,
    max: 0x7F
};

// Brightness for active LEDs
const activeBrightness = ledLevels[engine.getSetting("activeBrightness")] ?? ledLevels.max;

// Brightness for inactive LEDs
const inactiveBrightness = ledLevels[engine.getSetting("inactiveBrightness")] ?? ledLevels.low;

// Brightness for VU meters
const vuBrightness = ledLevels[engine.getSetting("vuBrightness")] ?? ledLevels.max;

// VU meter LED segment decay speed
const vuDecayFactor = engine.getSetting("vuDecayFactor") || 0;

// Use crossfader calibration data stored in device memory
const crossfaderCalibration = !!engine.getSetting("crossfaderCalibration");

// Manual crossfader calibration overrides
const crossfaderCalibrationOverride = [
    engine.getSetting("crossfaderCalibrationLeft") || 0,
    engine.getSetting("crossfaderCalibrationRight") || 4097
];

// Use latching mode button
const modeButtonLatch = !!engine.getSetting("modeButtonLatch");

// Invert primary and secondary button functions
const invertControls = !!engine.getSetting("invertControls");

class TraktorZ1Class {
    constructor() {
        this.controller = new HIDController();

        // Modifier states
        this.modePressed = false;
        this.modeLatched = false;

        // VU meter connections
        this.vuLeftConnection = {};
        this.vuRightConnection = {};

        // VU meter segment names and thresholds
        this.vuMeterThresholds = [
            {segment: "vu-30", threshold: 1 / 7},
            {segment: "vu-15", threshold: 2 / 7},
            {segment: "vu-6", threshold: 3 / 7},
            {segment: "vu-3", threshold: 4 / 7},
            {segment: "vu0", threshold: 5 / 7},
            {segment: "vu3", threshold: 6 / 7},
            {segment: "vu6", threshold: 7 / 7}
        ];

        // Stereo meter brightness data
        this.vuMeterChannel1 = [];
        this.vuMeterChannel2 = [];

        // Calibration data
        this.rawCalibration = {};
        this.calibration = null;
    }

    init(_id) {
        this.id = _id;

        this.calibrate();
        this.registerInputPackets();
        this.registerOutputPackets();
        this.readCurrentPosition();
        this.enableSoftTakeover();

        console.log(this.id + " initialized");
    }

    registerInputPackets() {
        const InputReport0x01 = new HIDPacket("InputReport0x01", 0x01, this.inputReportCallback.bind(this));

        // Mode button
        this.registerInputButton(InputReport0x01, "[ControlX]", "!mode", 0x1D, 0x02, this.modeHandler.bind(this));

        // Select FX / play and PFL / cue button handlers based on user preferences
        if (invertControls) {
            this.registerInputButton(InputReport0x01, "[Channel1]", "!cue", 0x1D, 0x10, this.cueHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel2]", "!cue", 0x1D, 0x01, this.cueHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel1]", "!play", 0x1D, 0x04, this.playHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel2]", "!play", 0x1D, 0x08, this.playHandler.bind(this));
        } else {
            this.registerInputButton(InputReport0x01, "[Channel1]", "!pfl", 0x1D, 0x10, this.headphoneHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel2]", "!pfl", 0x1D, 0x01, this.headphoneHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel1]", "!fx", 0x1D, 0x04, this.fxHandler.bind(this));
            this.registerInputButton(InputReport0x01, "[Channel2]", "!fx", 0x1D, 0x08, this.fxHandler.bind(this));
        }

        // EQ knobs
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel1]_Effect1]", "parameter3", 0x03, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel1]_Effect1]", "parameter2", 0x05, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel1]_Effect1]", "parameter1", 0x07, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel2]_Effect1]", "parameter3", 0x0D, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel2]_Effect1]", "parameter2", 0x0F, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[EqualizerRack1_[Channel2]_Effect1]", "parameter1", 0x11, 0xFFFF, this.parameterHandler.bind(this));

        // FX knobs
        this.registerInputScaler(InputReport0x01, "[QuickEffectRack1_[Channel1]]", "super1", 0x09, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[QuickEffectRack1_[Channel2]]", "super1", 0x13, 0xFFFF, this.parameterHandler.bind(this));

        // Gain knobs
        this.registerInputScaler(InputReport0x01, "[Channel1]", "pregain", 0x01, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[Channel2]", "pregain", 0x0B, 0xFFFF, this.parameterHandler.bind(this));

        // Headphone mix
        this.registerInputScaler(InputReport0x01, "[Master]", "headMix", 0x15, 0xFFFF, this.parameterHandler.bind(this));

        // Volume faders
        this.registerInputScaler(InputReport0x01, "[Channel1]", "volume", 0x17, 0xFFFF, this.parameterHandler.bind(this));
        this.registerInputScaler(InputReport0x01, "[Channel2]", "volume", 0x19, 0xFFFF, this.parameterHandler.bind(this));

        // Crossfader
        this.registerInputScaler(InputReport0x01, "[Master]", "crossfader", 0x1B, 0xFFFF, this.crossfaderHandler.bind(this));

        this.controller.registerInputPacket(InputReport0x01);
    }

    registerOutputPackets() {
        const OutputReport0x80 = new HIDPacket("OutputReport0x80", 0x80);

        OutputReport0x80.addOutput("[ControlX]", "mode", 0x13, "B");

        if (invertControls) {
            OutputReport0x80.addOutput("[Channel1]", "cue_indicator", 0x0F, "B");
            OutputReport0x80.addOutput("[Channel2]", "cue_indicator", 0x10, "B");
        } else {
            OutputReport0x80.addOutput("[Channel1]", "pfl", 0x0F, "B");
            OutputReport0x80.addOutput("[Channel2]", "pfl", 0x10, "B");
        }

        OutputReport0x80.addOutput("[Channel1]", "play_indicator", 0x11, "B");
        OutputReport0x80.addOutput("[Channel2]", "play_indicator", 0x14, "B");

        OutputReport0x80.addOutput("[QuickEffectRack1_[Channel1]]", "enabled", 0x12, "B");
        OutputReport0x80.addOutput("[QuickEffectRack1_[Channel2]]", "enabled", 0x15, "B");

        OutputReport0x80.addOutput("[Channel1]", "vu-30", 0x01, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu-15", 0x02, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu-6", 0x03, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu-3", 0x04, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu0", 0x05, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu3", 0x06, "B");
        OutputReport0x80.addOutput("[Channel1]", "vu6", 0x07, "B");

        OutputReport0x80.addOutput("[Channel2]", "vu-30", 0x08, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu-15", 0x09, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu-6", 0x0A, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu-3", 0x0B, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu0", 0x0C, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu3", 0x0D, "B");
        OutputReport0x80.addOutput("[Channel2]", "vu6", 0x0E, "B");

        this.controller.registerOutputPacket(OutputReport0x80);

        if (invertControls) {
            engine.makeConnection("[Channel1]", "cue_indicator", this.outputHandler.bind(this));
            engine.makeConnection("[Channel2]", "cue_indicator", this.outputHandler.bind(this));
            engine.makeConnection("[Channel1]", "play_indicator", this.outputHandler.bind(this));
            engine.makeConnection("[Channel2]", "play_indicator", this.outputHandler.bind(this));
        } else {
            engine.makeConnection("[Channel1]", "pfl", this.outputHandler.bind(this));
            engine.makeConnection("[Channel2]", "pfl", this.outputHandler.bind(this));
            engine.makeConnection("[QuickEffectRack1_[Channel1]]", "enabled", this.outputHandler.bind(this));
            engine.makeConnection("[QuickEffectRack1_[Channel2]]", "enabled", this.outputHandler.bind(this));
        }

        this.vuLeftConnection = engine.makeUnbufferedConnection("[Channel1]", "vu_meter", this.vuMeterHandler.bind(this));
        this.vuRightConnection = engine.makeUnbufferedConnection("[Channel2]", "vu_meter", this.vuMeterHandler.bind(this));

        this.lightDeck(false);
    }

    calibrate() {
        this.rawCalibration.faders = new Uint8Array(0x20 * 2);
        this.rawCalibration.faders.set(new Uint8Array(controller.getFeatureReport(0xD1)), 0x00);
        this.rawCalibration.faders.set(new Uint8Array(controller.getFeatureReport(0xD2)), 0x20);
        this.calibration = this.parseRawCalibration();
    }

    parseRawCalibration() {
        return {
            crossfader: this.parseCrossfaderCalibration(0x3C),
        };
    }

    parseCrossfaderCalibration(index) {
        const data = this.rawCalibration.faders;
        return {
            min: this.parseUint16Le(data, index),
            max: this.parseUint16Le(data, index+2),
        };
    }

    parseUint16Le(data, index) {
        return data[index] + (data[index+1]<<8);
    }

    readCurrentPosition() {
        // Sync on-screen controls with controller knob positions
        const report0x01 = new Uint8Array(controller.getInputReport(0x01));
        // The first packet is ignored by HIDController
        this.controller.parsePacket([0x01, ...Array.from(report0x01.map(x => x ^ 0xFF))]);
        this.controller.parsePacket([0x01, ...Array.from(report0x01)]);
    }

    enableSoftTakeover() {
        // Soft takeover for all knobs and faders
        engine.softTakeover("[EqualizerRack1_[Channel1]_Effect1]", "parameter3", true);
        engine.softTakeover("[EqualizerRack1_[Channel1]_Effect1]", "parameter2", true);
        engine.softTakeover("[EqualizerRack1_[Channel1]_Effect1]", "parameter1", true);

        engine.softTakeover("[EqualizerRack1_[Channel2]_Effect1]", "parameter3", true);
        engine.softTakeover("[EqualizerRack1_[Channel2]_Effect1]", "parameter2", true);
        engine.softTakeover("[EqualizerRack1_[Channel2]_Effect1]", "parameter1", true);

        engine.softTakeover("[QuickEffectRack1_[Channel1]]", "super1", true);
        engine.softTakeover("[QuickEffectRack1_[Channel2]]", "super1", true);

        engine.softTakeover("[Channel1]", "pregain", true);
        engine.softTakeover("[Channel2]", "pregain", true);

        engine.softTakeover("[Master]", "headMix", true);

        engine.softTakeover("[Channel1]", "volume", true);
        engine.softTakeover("[Channel2]", "volume", true);

        engine.softTakeover("[Master]", "crossfader", true);
    }

    registerInputButton(inputReport, group, name, offset, bitmask, callback) {
        inputReport.addControl(group, name, offset, "B", bitmask);
        inputReport.setCallback(group, name, callback);
    }

    registerInputScaler(inputReport, group, name, offset, bitmask, callback) {
        inputReport.addControl(group, name, offset, "H", bitmask);
        inputReport.setCallback(group, name, callback);
    }

    modeHandler(field) {
        if (field.value === 0 && this.modeLatched) {
            // Nothing to do if mode button is latched
            return;
        } else if (field.value === 1 && modeButtonLatch) {
            // Toggle mode latching
            this.modeLatched = !this.modeLatched;
        }
        this.modePressed = field.value;
        this.outputHandler(field.value, field.group, "mode");
    }

    cueHandler(field) {
        if (field.value === 1) {
            if (this.modePressed || this.modeLatched) {
                // Toggle PFL as secondary function
                script.toggleControl(field.group, "pfl");
                this.outputHandler(field.value, field.group, "pfl");
            } else {
                engine.setValue(field.group, "cue_gotoandstop", field.value);
                this.outputHandler(field.value, field.group, "cue_indicator");
            }
        }
    }

    headphoneHandler(field) {
        if (field.value === 1) {
            if (this.modePressed || this.modeLatched) {
            // Seek to cue as secondary function
            engine.setValue(field.group, "cue_gotoandstop", field.value);
            this.controller.setOutput(field.group, "pfl", activeBrightness, true);
            } else {
                script.toggleControl(field.group, "pfl");
            }
        }
    }

    playHandler(field) {
        if (field.value === 1) {
            if (this.modePressed || this.modeLatched) {
                // Use blue LED as a momentary indicator
                this.controller.setOutput(field.group, "play_indicator", ledLevels.off, true);
                this.controller.setOutput(`[QuickEffectRack1_${field.group}]`, "enabled", activeBrightness, true);
                script.toggleControl(`[QuickEffectRack1_${field.group}]`, "enabled");
            } else {
                script.toggleControl(field.group, "play");
            }
        } else {
            // Always reset momentary blue indicator LED
            this.controller.setOutput(`[QuickEffectRack1_${field.group}]`, "enabled", ledLevels.off, true);

            // Restore correct red LED state
            const ledBrightness = engine.getValue(field.group, "play") ? activeBrightness : inactiveBrightness;
            this.controller.setOutput(field.group, "play_indicator", ledBrightness, true);
        }
    }

    fxHandler(field) {
        if (field.value === 1) {
            if (this.modePressed || this.modeLatched) {
                if (engine.getValue(field.group, "track_loaded")) {
                    // Use red LED as a momentary indicator
                    this.controller.setOutput(`[QuickEffectRack1_${field.group}]`, "enabled", ledLevels.off, true);
                    this.controller.setOutput(field.group, "play_indicator", activeBrightness, true);
                    script.toggleControl(field.group, "play");
                }
            } else {
                script.toggleControl(`[QuickEffectRack1_${field.group}]`, "enabled");
            }
        } else {
            // Always reset momentary red indicator LED
            this.controller.setOutput(field.group, "play_indicator", ledLevels.off, true);

            // Restore correct blue LED state
            const ledBrightness = engine.getValue(`[QuickEffectRack1_${field.group}]`, "enabled") ? activeBrightness : inactiveBrightness;
            this.controller.setOutput(`[QuickEffectRack1_${field.group}]`, "enabled", ledBrightness, true);
        }
    }

    vuMeterHandler(value, group, _key) {
        if (vuBrightness === 0) {
            // Nothing to do if meters are disabled
            return;
        }

        // Select correct data array based on channel
        const vuData = (group === "[Channel1]") ? this.vuMeterChannel1 : this.vuMeterChannel2;

        for (let i = 0; i < this.vuMeterThresholds.length; ++i) {
            // Avoid spamming HID by only sending last LED update
            const last = i === (this.vuMeterThresholds.length - 1);

            // Fill the data array
            vuData[i] = vuData[i] || {segment: this.vuMeterThresholds[i].segment, brightness: 0};

            // Light or fade segment according to brightness and decay settings
            if (value >= this.vuMeterThresholds[i].threshold) {
                vuData[i].brightness = vuBrightness;
            } else {
                vuData[i].brightness = Math.max(0, Math.round(vuData[i].brightness * vuDecayFactor) - 1);
            }
            this.controller.setOutput(group, vuData[i].segment, vuData[i].brightness, last);
        }
    }

    parameterHandler(field) {
        engine.setParameter(field.group, field.name, field.value / 4095);
    }

    crossfaderHandler(field) {
        // Extra safety margins for both on-device and manual crossfader calibration values
        const safeMargins = 5;

        // Use manual overrides if on-device calibration is not enabled
        const min = crossfaderCalibration ? this.calibration.crossfader.min : crossfaderCalibrationOverride[0];
        const max = crossfaderCalibration ? this.calibration.crossfader.max : crossfaderCalibrationOverride[1];
        const value = script.absoluteLin(field.value, 0, 1, min + safeMargins, max - safeMargins);
        engine.setParameter(field.group, field.name, value);
    }

    outputHandler(value, group, key) {
        let ledValue;

        if (value === 0 || value === false) {
            // Inactive brightness value
            ledValue = inactiveBrightness;
        } else if (value === 1 || value === true) {
            // Active brightness value
            ledValue = activeBrightness;
        }
        this.controller.setOutput(group, key, ledValue, true);
    }

    lightDeck(switchOff) {
        let softLight = inactiveBrightness;
        let fullLight = activeBrightness;
        let ledBrightness;

        if (switchOff) {
            softLight = ledLevels.off;
            fullLight = ledLevels.off;
        }

        this.controller.setOutput("[ControlX]", "mode", softLight, true);

        if (invertControls) {
            this.controller.setOutput("[Channel1]", "cue_indicator", softLight, true);
            this.controller.setOutput("[Channel2]", "cue_indicator", softLight, true);

            ledBrightness = engine.getValue("[Channel1]", "play") ? fullLight : softLight;
            this.controller.setOutput("[Channel1]", "play_indicator", ledBrightness, true);
            ledBrightness = engine.getValue("[Channel2]", "play") ? fullLight : softLight;
            this.controller.setOutput("[Channel2]", "play_indicator", ledBrightness, true);
        } else {
            ledBrightness = engine.getValue("[Channel1]", "pfl") ? fullLight : softLight;
            this.controller.setOutput("[Channel1]", "pfl", ledBrightness, true);
            ledBrightness = engine.getValue("[Channel2]", "pfl") ? fullLight : softLight;
            this.controller.setOutput("[Channel2]", "pfl", ledBrightness, true);

            ledBrightness = engine.getValue("[QuickEffectRack1_[Channel1]]", "enabled") ? fullLight : softLight;
            this.controller.setOutput("[QuickEffectRack1_[Channel1]]", "enabled", ledBrightness, true);
            ledBrightness = engine.getValue("[QuickEffectRack1_[Channel2]]", "enabled") ? fullLight : softLight;
            this.controller.setOutput("[QuickEffectRack1_[Channel2]]", "enabled", ledBrightness, true);
        }
    }

    inputReportCallback(packet, data) {
        for (const name in data) {
            if (Object.hasOwnProperty.call(data, name)) {
                this.controller.processButton(data[name]);
            }
        }
    }

    shutdown() {
        // Deactivate all LEDs
        this.lightDeck(true);
        console.log(this.id + " shut down");
    }

    incomingData(data, length) {
        this.controller.parsePacket(data, length);
    }
}

var TraktorZ1 = new TraktorZ1Class;  // eslint-disable-line no-var, no-unused-vars
