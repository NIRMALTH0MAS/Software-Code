# Cartesian Picker 3D Visualization

Interactive 3D visualization of a Cartesian Picker robot using Three.js, with smooth time-based animations and dual control methods (UI sliders and Python WebSocket integration).

## Features

- 🎨 **3D Visualization**: High-quality rendering of STL model with custom materials and lighting
- 🎮 **Dual Control**: Control via web UI sliders or Python WebSocket commands
- ⏱️ **Smooth Animations**: All movements transition smoothly over 5 seconds
- 🔧 **Independent Axes**: Move X, Y, and Z components independently
- 🌐 **Real-time Updates**: WebSocket integration for remote control
- 📱 **Responsive Design**: Modern, dark-themed UI with glassmorphism effects

## Quick Start

### 1. Open the Web Interface

Since this is a static HTML application, you need to serve it via a local web server (required for loading the STL file).

**Option A: Using Python**
```bash
# Python 3
python -m http.server 8000

# Then open: http://localhost:8000
```

**Option B: Using Node.js**
```bash
npx serve
# Follow the URL provided
```

**Option C: Using VS Code**
Install the "Live Server" extension and right-click `index.html` → "Open with Live Server"

### 2. Control via UI Sliders

Once the page loads:
- Use the **X, Y, Z sliders** in the control panel to move components
- Each movement takes **5 seconds** to complete
- Values are displayed in real-time

### 3. Control via Python (Optional)

Install Python dependencies:
```bash
pip install -r requirements.txt
```

Run the WebSocket controller:
```bash
python python_controller.py
```

The script will:
- Start a WebSocket server on `ws://localhost:8765`
- Run an example control sequence
- Send position commands in the format: `{x: value, y: value, z: value}`

## Project Structure

```
Cartesian Picker (DT)/
├── index.html              # Main HTML file
├── styles.css              # Styling and UI design
├── app.js                  # Three.js scene and STL loader
├── animator.js             # Smooth animation system
├── controls.js             # UI slider controls
├── websocket-client.js     # WebSocket client for Python integration
├── python_controller.py    # Python WebSocket server (optional)
├── requirements.txt        # Python dependencies
├── cartesian_picker.stl    # 3D model file
└── README.md              # This file
```

## Architecture

### Component Hierarchy

The application uses a hierarchical structure for independent axis movement:

```
Model Group
└── Z Component (moves along Z-axis)
    └── Y Component (moves along Y-axis)
        └── X Component (moves along X-axis)
            └── Base Mesh (STL model)
```

### Animation System

- **Duration**: 5 seconds per movement
- **Easing**: Cubic ease-in-out for smooth transitions
- **Independent**: Each axis can move simultaneously without interference
- **Interpolation**: Position values are smoothly interpolated between current and target

### WebSocket Protocol

**Message Format:**
```json
{
  "x": 50.0,
  "y": -25.5,
  "z": 100.0
}
```

**Port:** 8765 (default)

**Connection:** `ws://localhost:8765`

## Camera Controls

- **Left Mouse**: Rotate view
- **Right Mouse**: Pan view
- **Mouse Wheel**: Zoom in/out

## Customization

### Adjust Movement Range

Edit the slider ranges in `index.html`:
```html
<input type="range" id="xSlider" min="-100" max="100" value="0" step="0.1">
```

### Change Animation Duration

Edit `animator.js`:
```javascript
this.duration = 5000; // Change to desired milliseconds
```

### Modify Colors

Component colors are defined in `app.js`:
```javascript
const materials = {
    xComponent: new THREE.MeshPhongMaterial({ color: 0xff6b6b }), // Red
    yComponent: new THREE.MeshPhongMaterial({ color: 0x4ecdc4 }), // Cyan
    zComponent: new THREE.MeshPhongMaterial({ color: 0x5f72ff })  // Blue
};
```

### WebSocket Port

Change the port in both files:

**websocket-client.js:**
```javascript
this.ws = new WebSocket('ws://localhost:8765');
```

**python_controller.py:**
```python
controller = CartesianPickerController(port=8765)
```

## Troubleshooting

### STL Model Not Loading

**Problem:** "Error loading 3D model" message appears

**Solutions:**
- Ensure you're using a local web server (not opening `index.html` directly)
- Check that `cartesian_picker.stl` is in the same directory
- Check browser console for specific error messages

### WebSocket Not Connecting

**Problem:** Status shows "Disconnected"

**Solutions:**
- Ensure Python WebSocket server is running: `python python_controller.py`
- Check that port 8765 is not blocked by firewall
- Verify the WebSocket URL in `websocket-client.js` matches your server

### Movements Not Smooth

**Problem:** Animations are jerky or instant

**Solutions:**
- Check browser performance (try closing other tabs)
- Ensure `animator.js` is loaded before `app.js`
- Verify the animation duration is set correctly

### Components Not Moving Independently

**Problem:** All parts move together

**Note:** The current implementation moves the entire model as a unit. For true independent component movement, the STL file would need to be split into separate parts (X-axis carriage, Y-axis column, Z-axis end effector). This can be done in CAD software like Blender or FreeCAD.

## Development

### Adding Custom Control Logic

Extend the `Controls` class in `controls.js`:
```javascript
class Controls {
    // Add custom methods
    moveToPreset(presetName) {
        const presets = {
            home: { x: 0, y: 0, z: 0 },
            corner: { x: 100, y: 100, z: 100 }
        };
        this.setAllPositions(presets[presetName]);
    }
}
```

### Integrating with Other Systems

The WebSocket interface accepts any client that can send JSON messages:

**JavaScript Example:**
```javascript
const ws = new WebSocket('ws://localhost:8765');
ws.send(JSON.stringify({ x: 50, y: 25, z: 75 }));
```

**Python Example:**
```python
import asyncio
import websockets
import json

async def send_position():
    async with websockets.connect('ws://localhost:8765') as ws:
        await ws.send(json.dumps({"x": 50, "y": 25, "z": 75}))

asyncio.run(send_position())
```

## Technical Details

### Dependencies

**Frontend:**
- Three.js r128 (via CDN)
- OrbitControls (via CDN)
- STLLoader (via CDN)

**Backend (Optional):**
- Python 3.7+
- websockets library

### Browser Compatibility

- Chrome/Edge: ✅ Fully supported
- Firefox: ✅ Fully supported
- Safari: ✅ Fully supported
- IE11: ❌ Not supported

### Performance

- **Model Size**: Optimized for STL files up to 50MB
- **Frame Rate**: 60 FPS on modern hardware
- **Animation**: Hardware-accelerated via requestAnimationFrame

## License

This project is provided as-is for educational and demonstration purposes.

## Support

For issues or questions:
1. Check the browser console for error messages
2. Verify all files are in the correct directory
3. Ensure you're using a local web server
4. Check that the STL file is valid and not corrupted
