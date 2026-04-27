// app.js - Main Three.js application
let scene, camera, renderer, controls;
let currentModel, components;
let animator, uiControls, wsClient, uiManager;
let axisLabels = [];
let glbLoader;
let isGLBMode = false; // Track if current model is GLB (full interaction) or STL (visualization only)

// Initialize the application
function init() {
    // Create scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0a);
    scene.fog = new THREE.Fog(0x0a0a0a, 500, 2000);

    // Create camera (Z-up orientation like Blender)
    const canvas = document.getElementById('canvas3d');
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    camera = new THREE.PerspectiveCamera(45, width / height, 1, 5000);
    camera.position.set(400, -400, 300); // Angle view for Z-up
    camera.up.set(0, 0, 1); // Z-axis is UP
    camera.lookAt(0, 0, 0);

    // Global default up for Three.js
    THREE.Object3D.DefaultUp.set(0, 0, 1);

    // Create renderer
    renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        antialias: true,
        alpha: true
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // Add orbit controls (Z-up)
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.minDistance = 100;
    controls.maxDistance = 1000;
    controls.target.set(0, 0, 0);
    controls.screenSpacePanning = false; // Important for Z-up

    // Add lights
    setupLighting();

    // Add grid helper on XY plane (Z-up orientation)
    // In THREE, GridHelper is always on the XZ plane by default. 
    // To make it on XY plane, we rotate it 90 degrees on X axis.
    const gridHelper = new THREE.GridHelper(500, 50, 0x00d4ff, 0x222222);
    gridHelper.rotation.x = Math.PI / 2; // Flat on the desk (XY plane)
    gridHelper.position.z = 0;
    scene.add(gridHelper);

    // Add axes helper
    const axesHelper = new THREE.AxesHelper(150);
    scene.add(axesHelper);

    // Add axis labels
    addAxisLabels();

    // Initialize animator
    animator = new Animator();

    // Initialize GLB loader
    glbLoader = new GLBLoader(scene, onGLBLoaded);

    // Initialize UI Manager
    uiManager = new UIManager();

    // Initialize WebSocket client (will auto-connect)
    wsClient = new WebSocketClient((positions) => {
        // Handle position updates from Python
        if (uiControls) {
            uiControls.setAllPositions(positions);
        }
    });

    // Show message to load a file
    const loadingElement = document.getElementById('loading');
    loadingElement.innerHTML = '<p>Please load a GLB or STL file to begin</p>';
    loadingElement.style.display = 'flex';

    // Handle window resize
    window.addEventListener('resize', onWindowResize, false);

    // Start animation loop
    animate();
}

function setupLighting() {
    // Ambient light for overall illumination
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    // Main directional light
    const mainLight = new THREE.DirectionalLight(0xffffff, 0.8);
    mainLight.position.set(200, 300, 200);
    mainLight.castShadow = true;
    mainLight.shadow.camera.left = -200;
    mainLight.shadow.camera.right = 200;
    mainLight.shadow.camera.top = 200;
    mainLight.shadow.camera.bottom = -200;
    mainLight.shadow.mapSize.width = 2048;
    mainLight.shadow.mapSize.height = 2048;
    scene.add(mainLight);

    // Fill lights from different angles
    const fillLight1 = new THREE.DirectionalLight(0x00d4ff, 0.3);
    fillLight1.position.set(-200, 100, -200);
    scene.add(fillLight1);

    const fillLight2 = new THREE.DirectionalLight(0xff6b6b, 0.2);
    fillLight2.position.set(200, -100, 200);
    scene.add(fillLight2);

    // Hemisphere light for natural gradient
    const hemiLight = new THREE.HemisphereLight(0x00d4ff, 0x302b63, 0.3);
    scene.add(hemiLight);
}



function onWindowResize() {
    const canvas = document.getElementById('canvas3d');
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

function animate() {
    requestAnimationFrame(animate);

    // Update controls
    controls.update();

    // Render scene
    renderer.render(scene, camera);
}

// Start the application when DOM is loaded
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

// Add 3D axis labels
function addAxisLabels() {
    const createTextLabel = (text, position, color) => {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 256;
        canvas.height = 128;

        context.fillStyle = color;
        context.font = 'Bold 60px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(text, 128, 64);

        const texture = new THREE.CanvasTexture(canvas);
        const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
        const sprite = new THREE.Sprite(spriteMaterial);
        sprite.position.copy(position);
        sprite.scale.set(40, 20, 1);

        return sprite;
    };

    // Clear existing labels
    axisLabels.forEach(label => scene.remove(label));
    axisLabels = [];

    // X-axis label (Red)
    const xLabel = createTextLabel('X', new THREE.Vector3(180, 0, 0), '#ff6b6b');
    scene.add(xLabel);
    axisLabels.push(xLabel);

    // Y-axis label (Green/Cyan)
    const yLabel = createTextLabel('Y', new THREE.Vector3(0, 180, 0), '#4ecdc4');
    scene.add(yLabel);
    axisLabels.push(yLabel);

    // Z-axis label (Blue)
    const zLabel = createTextLabel('Z', new THREE.Vector3(0, 0, 180), '#5f72ff');
    scene.add(zLabel);
    axisLabels.push(zLabel);
}

// Auto-scale view based on model size
function autoScaleView(modelSize) {
    const maxDim = Math.max(modelSize.x, modelSize.y, modelSize.z);
    const fov = camera.fov * (Math.PI / 180);
    let cameraDistance = Math.abs(maxDim / Math.sin(fov / 2));
    cameraDistance *= 1.5; // Add some padding

    // Update camera position
    const direction = camera.position.clone().normalize();
    camera.position.copy(direction.multiplyScalar(cameraDistance));

    // Update controls
    controls.minDistance = cameraDistance * 0.5;
    controls.maxDistance = cameraDistance * 3;
    controls.update();

    console.log(`Auto-scaled view for model size: ${maxDim.toFixed(2)}`);
}

// Callback when GLB is loaded
function onGLBLoaded(data) {
    console.log('GLB loaded callback', data);
    isGLBMode = true;

    // Remove old model
    if (currentModel) {
        scene.remove(currentModel);
    }

    // Add new model
    currentModel = data.model;
    window.currentModel = currentModel; // Expose for Controls.js access

    scene.add(currentModel);

    // Update component tree UI
    updateComponentTreeFromGLB(data.tree);

    // Use the components identified by GLBLoader
    components = {
        x: glbLoader.identifiedComponents.x,
        y: glbLoader.identifiedComponents.y,
        z: glbLoader.identifiedComponents.z
    };

    console.log('Mapped components for control:', components);

    // Initialize/update controls
    if (!uiControls) {
        uiControls = new Controls(animator, components);
    } else {
        uiControls.components = components;
    }

    // Auto-scale view
    autoScaleView(data.size);
}

// Update component tree from GLB hierarchy
function updateComponentTreeFromGLB(tree) {
    const treeContainer = document.getElementById('componentTree');
    const treeHTML = glbLoader.generateTreeHTML(tree);
    treeContainer.innerHTML = treeHTML;

    // Reinitialize tree interactions
    if (uiManager) {
        uiManager.initializeComponentTree();
    }
}

// Find component by axis name in tree
function findComponentByAxis(axis, tree) {
    const axisLower = axis.toLowerCase();

    function search(nodes) {
        for (const node of nodes) {
            const nameLower = node.name.toLowerCase();
            if (nameLower.includes(axisLower) &&
                (nameLower.includes('axis') || nameLower.includes('component'))) {
                return node.name;
            }
            if (node.children && node.children.length > 0) {
                const found = search(node.children);
                if (found) return found;
            }
        }
        return null;
    }

    return search(tree);
}

// Load GLB from file buffer
window.loadGLBFromBuffer = function (arrayBuffer, filename) {
    console.log(`Loading GLB file: ${filename}`);
    isGLBMode = true;
    glbLoader.loadFromBuffer(arrayBuffer, filename);
};



// Highlight component in 3D scene
window.highlightComponent = function (componentName) {
    console.log('Highlighting component:', componentName);

    // Reset all materials to default
    if (components) {
        Object.values(components).forEach(comp => {
            comp.traverse(child => {
                if (child.isMesh && child.material) {
                    child.material.emissive = new THREE.Color(0x000000);
                }
            });
        });
    }

    // Highlight selected component
    if (componentName && components[componentName]) {
        components[componentName].traverse(child => {
            if (child.isMesh && child.material) {
                child.material.emissive = new THREE.Color(0x00d4ff);
                child.material.emissiveIntensity = 0.3;
            }
        });
    }
};
