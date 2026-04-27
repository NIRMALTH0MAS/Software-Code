// Controls.js - UI slider controls, manual inputs, and axis correction
class Controls {
    constructor(animator, components) {
        this.animator = animator;
        this.components = components;

        // Get slider elements
        this.sliders = {
            x: document.getElementById('xSlider'),
            y: document.getElementById('ySlider'),
            z: document.getElementById('zSlider')
        };

        // Get manual input elements
        this.inputs = {
            x: document.getElementById('xInput'),
            y: document.getElementById('yInput'),
            z: document.getElementById('zInput')
        };

        // Get min/max limit inputs
        this.minInputs = {
            x: document.getElementById('xMin'),
            y: document.getElementById('yMin'),
            z: document.getElementById('zMin')
        };

        this.maxInputs = {
            x: document.getElementById('xMax'),
            y: document.getElementById('yMax'),
            z: document.getElementById('zMax')
        };

        // Get value display elements
        this.valueDisplays = {
            x: document.getElementById('xValue'),
            y: document.getElementById('yValue'),
            z: document.getElementById('zValue')
        };



        // Map UI axes to model internal position properties
        // Per user request: UI Z (Up/Down) moves model Y, UI Y (Push/Pull) moves model Z
        this.propertyMap = {
            x: 'x',
            y: 'z',
            z: 'y'
        };

        this.initializeEventListeners();
    }

    initializeEventListeners() {
        // Slider event listeners
        Object.keys(this.sliders).forEach(axis => {
            const slider = this.sliders[axis];

            // Update on input (while dragging)
            slider.addEventListener('input', (e) => {
                const percent = parseFloat(e.target.value);

                // Show 0-100 percent in the UI
                this.updateValueDisplay(axis, percent);
                this.inputs[axis].value = percent.toFixed(1);
            });

            // Trigger animation on change (when released)
            slider.addEventListener('change', (e) => {
                const percent = parseFloat(e.target.value);
                const min = parseFloat(this.minInputs[axis].value);
                const max = parseFloat(this.maxInputs[axis].value);

                const physicalValue = min + (percent / 100) * (max - min);
                this.moveComponent(axis, physicalValue);
            });
        });

        // Manual input event listeners (0-100 scale)
        Object.keys(this.inputs).forEach(axis => {
            const input = this.inputs[axis];

            input.addEventListener('change', (e) => {
                let percent = parseFloat(e.target.value);

                // Clamp to 0-100
                percent = Math.max(0, Math.min(100, percent));

                // Update input to clamped value
                input.value = percent.toFixed(1);
                this.sliders[axis].value = percent;
                this.updateValueDisplay(axis, percent);

                // Map to physical for movement
                const min = parseFloat(this.minInputs[axis].value);
                const max = parseFloat(this.maxInputs[axis].value);
                const physicalValue = min + (percent / 100) * (max - min);

                // Move component
                this.moveComponent(axis, physicalValue);
            });

            // Update on Enter key
            input.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    input.blur(); // Trigger change event
                }
            });
        });

        // Min/Max limit event listeners (just update the labels/values)
        ['x', 'y', 'z'].forEach(axis => {
            const updateMapping = () => {
                const percent = parseFloat(this.sliders[axis].value);
                const min = parseFloat(this.minInputs[axis].value);
                const max = parseFloat(this.maxInputs[axis].value);

                // Map percent to technical value for movement
                const physicalValue = min + (percent / 100) * (max - min);

                // Update midpoint label in UI
                const midpoint = (min + max) / 2;
                const label = this.minInputs[axis].parentElement.querySelector('span');
                if (label) label.textContent = midpoint.toFixed(2);

                this.moveComponent(axis, physicalValue);
            };

            this.minInputs[axis].addEventListener('change', updateMapping);
            this.maxInputs[axis].addEventListener('change', updateMapping);
        });


    }





    updateValueDisplay(axis, value) {
        if (this.valueDisplays[axis]) {
            this.valueDisplays[axis].textContent = value.toFixed(2);
        }
    }

    moveComponent(axis, targetPosition) {
        const component = this.components[axis];

        if (!component) {
            console.warn(`Component for ${axis} axis not found`);
            return;
        }

        // Use property mapping to ensure UI 'z' moves model 'y', etc.
        const targetProp = this.propertyMap[axis];

        this.animator.animateTo(axis, component, targetPosition, (currentPos) => {
            // Internal update if needed, but UI is handled by listeners or setPosition
        }, targetProp);
    }


    // Set position programmatically (updates UI and 3D)
    // - value: the value to set (0-100 by default)
    // - mode: 'percent' (0-100) or 'physical' (e.g. -3.1)
    setPosition(axis, value, mode = 'percent') {
        const min = parseFloat(this.minInputs[axis].value);
        const max = parseFloat(this.maxInputs[axis].value);

        let percent, physicalValue;

        if (mode === 'percent') {
            percent = Math.max(0, Math.min(100, value));
            physicalValue = min + (percent / 100) * (max - min);
        } else {
            // mode === 'physical'
            const realMin = Math.min(min, max);
            const realMax = Math.max(min, max);
            physicalValue = Math.max(realMin, Math.min(realMax, value));

            const range = max - min;
            percent = range === 0 ? 0 : ((physicalValue - min) / range) * 100;
        }

        // Update UI components (0-100 scale)
        this.sliders[axis].value = percent;
        this.inputs[axis].value = percent.toFixed(1);
        this.updateValueDisplay(axis, percent);

        // Move component in 3D using physical unit
        this.moveComponent(axis, physicalValue);
    }

    // Set all positions at once (for WebSocket {x, y, z} messages)
    setAllPositions(positions) {
        const mode = positions.mode || 'percent'; // Default to percent if not specified

        if (positions.x !== undefined) {
            this.setPosition('x', positions.x, mode);
        }
        if (positions.y !== undefined) {
            this.setPosition('y', positions.y, mode);
        }
        if (positions.z !== undefined) {
            this.setPosition('z', positions.z, mode);
        }
    }

    // Reset all positions to midpoints
    reset() {
        ['x', 'y', 'z'].forEach(axis => {
            const min = parseFloat(this.minInputs[axis].value);
            const max = parseFloat(this.maxInputs[axis].value);
            const midpoint = (min + max) / 2;

            this.sliders[axis].value = 50; // Slider to 50%
            this.inputs[axis].value = "50.0";
            this.updateValueDisplay(axis, 50);
            this.moveComponent(axis, midpoint);
        });
    }

    // Get current slider values
    getCurrentPositions() {
        return {
            x: parseFloat(this.sliders.x.value),
            y: parseFloat(this.sliders.y.value),
            z: parseFloat(this.sliders.z.value)
        };
    }
}

// Export for use in other scripts
window.Controls = Controls;
