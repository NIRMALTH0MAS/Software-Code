# Testing Guide for Cartesian Picker 3D Visualization

## Quick Start Testing

### Step 1: Start the Web Server

The server should already be running. If not, run one of these commands:

**Option A: Using npx (Recommended)**
```bash
npx -y serve -l 8000
```

**Option B: Using Python (if installed)**
```bash
python -m http.server 8000
```

### Step 2: Open the Application

Open your web browser and navigate to:
```
http://localhost:8000
```

### Step 3: Test the 3D Visualization

#### Initial Load Test
- ✅ **Expected**: Loading spinner appears, then disappears when model loads
- ✅ **Expected**: 3D Cartesian Picker model is visible in the center
- ✅ **Expected**: Control panel is visible on the right side
- ✅ **Expected**: WebSocket status shows "Disconnected" (normal without Python server)

#### Camera Controls Test
- **Left Mouse Drag**: Rotate the view around the model
- **Right Mouse Drag**: Pan the view
- **Mouse Wheel**: Zoom in and out
- ✅ **Expected**: Smooth, responsive camera movement

#### Slider Controls Test

**Test X-Axis Movement:**
1. Move the X-axis slider to position 50
2. Observe the value display update to "50.00"
3. Watch the model smoothly transition over 5 seconds
4. ✅ **Expected**: Smooth animation, no jerky movement

**Test Y-Axis Movement:**
1. Move the Y-axis slider to position -50
2. Observe the value display update to "-50.00"
3. Watch the model smoothly transition over 5 seconds
4. ✅ **Expected**: Independent movement from X-axis

**Test Z-Axis Movement:**
1. Move the Z-axis slider to position 75
2. Observe the value display update to "75.00"
3. Watch the model smoothly transition over 5 seconds
4. ✅ **Expected**: Independent movement from X and Y axes

**Test Simultaneous Movement:**
1. Quickly move all three sliders to different positions
2. ✅ **Expected**: All three axes animate simultaneously and independently
3. ✅ **Expected**: Each animation takes exactly 5 seconds

### Step 4: Test Python Integration (Optional)

#### Install Python Dependencies
```bash
pip install websockets
```

#### Run the Python Controller
```bash
python python_controller.py
```

#### Expected Behavior
1. Python server starts on port 8765
2. Web interface status changes to "Connected" (green dot)
3. Python sends example position sequence
4. Model moves automatically according to Python commands
5. Each movement takes 5 seconds

#### Test Custom Python Commands

Create a simple test script:

```python
import asyncio
import websockets
import json

async def send_position(x, y, z):
    async with websockets.connect('ws://localhost:8765') as ws:
        await ws.send(json.dumps({"x": x, "y": y, "z": z}))
        print(f"Sent: x={x}, y={y}, z={z}")

# Test different positions
asyncio.run(send_position(100, 50, -25))
```

## Troubleshooting Checklist

### Model Not Loading
- [ ] Check browser console (F12) for errors
- [ ] Verify `cartesian_picker.stl` is in the same folder
- [ ] Ensure you're using a web server (not opening file directly)
- [ ] Try a different browser (Chrome, Firefox, Edge)

### Animations Not Smooth
- [ ] Check if browser is using hardware acceleration
- [ ] Close other browser tabs to free up resources
- [ ] Verify no console errors during animation
- [ ] Check that all JavaScript files loaded correctly

### WebSocket Not Connecting
- [ ] Verify Python server is running
- [ ] Check that port 8765 is not blocked
- [ ] Look for connection errors in browser console
- [ ] Ensure websockets library is installed: `pip install websockets`

### Sliders Not Working
- [ ] Check browser console for JavaScript errors
- [ ] Verify all script files are loaded (check Network tab in DevTools)
- [ ] Try refreshing the page (Ctrl+F5)

## Performance Benchmarks

### Expected Performance
- **Model Load Time**: < 3 seconds for 47MB STL file
- **Frame Rate**: 60 FPS during rotation
- **Animation Smoothness**: No visible stuttering during 5-second transitions
- **WebSocket Latency**: < 50ms from Python command to animation start

### Browser Console Tests

Open browser console (F12) and run these tests:

**Check if components are loaded:**
```javascript
console.log('Components:', components);
console.log('Animator:', animator);
console.log('Controls:', uiControls);
```

**Manually trigger animation:**
```javascript
uiControls.setPosition('x', 50);
```

**Check animation progress:**
```javascript
console.log('X-axis progress:', animator.getProgress('x'));
```

**Test WebSocket manually:**
```javascript
wsClient.send({x: 25, y: 50, z: 75});
```

## Visual Quality Checklist

- [ ] Model is centered in viewport
- [ ] Lighting shows model details clearly
- [ ] Shadows are visible (if enabled)
- [ ] Grid and axes helpers are visible
- [ ] Control panel has glassmorphism effect
- [ ] Sliders have colored thumbs matching their axis
- [ ] Value displays update in real-time
- [ ] Status badge shows connection state

## Feature Verification

### Core Features
- [x] STL model loading
- [x] 3D rendering with Three.js
- [x] Camera orbit controls
- [x] Three independent axis sliders
- [x] Real-time value displays
- [x] 5-second smooth animations
- [x] WebSocket client for Python integration
- [x] Auto-reconnect on WebSocket disconnect

### Advanced Features
- [x] Simultaneous multi-axis animation
- [x] Easing function (cubic ease-in-out)
- [x] Responsive UI design
- [x] Dark theme with gradients
- [x] Custom colored materials for components
- [x] Multiple light sources
- [x] Shadow rendering
- [x] Grid and axes helpers

## Known Limitations

1. **Component Separation**: The current implementation moves the entire model as a unit. For true independent component movement, the STL would need to be split into separate files (X-carriage, Y-column, Z-effector).

2. **WebSocket Server**: Requires Python and the websockets library. The web interface works fine without it using only the sliders.

3. **Browser Compatibility**: Requires a modern browser with WebGL support. IE11 is not supported.

## Success Criteria

The application is working correctly if:
- ✅ STL model loads and displays in 3D
- ✅ All three sliders control movement
- ✅ Animations take exactly 5 seconds
- ✅ Movements are smooth with no stuttering
- ✅ Multiple axes can move simultaneously
- ✅ Camera controls work (rotate, pan, zoom)
- ✅ WebSocket connects when Python server is running
- ✅ Python commands trigger animations
- ✅ UI is responsive and visually appealing

## Next Steps

After verifying basic functionality:
1. Experiment with different position values
2. Try the Python controller example
3. Create custom Python scripts for specific movement patterns
4. Adjust colors and materials in `app.js`
5. Modify animation duration in `animator.js`
6. Customize slider ranges in `index.html`

## Support

If you encounter issues:
1. Check this testing guide
2. Review the README.md file
3. Check browser console for errors
4. Verify all files are present in the directory
5. Ensure you're using a local web server
