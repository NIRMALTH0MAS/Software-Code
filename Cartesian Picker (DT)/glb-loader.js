// GLB Loader - Handles GLB/GLTF file loading with component hierarchy
class GLBLoader {
    constructor(scene, onLoadComplete) {
        this.scene = scene;
        this.onLoadComplete = onLoadComplete;
        this.loader = new THREE.GLTFLoader();
        this.componentMap = new Map(); // Maps component names to Three.js objects
        this.componentTree = []; // Hierarchical tree structure
        this.movableComponents = new Set(); // ONLY X/Y/Z allowed to move
    }

    load(url) {
        const loadingElement = document.getElementById('loading');
        loadingElement.style.display = 'flex';
        loadingElement.querySelector('p').textContent = 'Loading GLB Model...';

        this.loader.load(
            url,
            (gltf) => this.onLoad(gltf),
            (progress) => this.onProgress(progress),
            (error) => this.onError(error)
        );
    }

    loadFromBuffer(arrayBuffer, filename) {
        const loadingElement = document.getElementById('loading');
        loadingElement.style.display = 'flex';
        loadingElement.querySelector('p').textContent = `Loading ${filename}...`;

        try {
            // Convert ArrayBuffer to blob URL for GLTFLoader
            const blob = new Blob([arrayBuffer], { type: 'model/gltf-binary' });
            const url = URL.createObjectURL(blob);

            this.loader.load(
                url,
                (gltf) => {
                    URL.revokeObjectURL(url); // Clean up
                    this.onLoad(gltf);
                },
                (progress) => this.onProgress(progress),
                (error) => {
                    URL.revokeObjectURL(url);
                    this.onError(error);
                }
            );
        } catch (error) {
            this.onError(error);
        }
    }

    onLoad(gltf) {
        console.log('GLB model loaded successfully', gltf);

        const model = gltf.scene;

        // model.rotation.x = -Math.PI / 2;
        model.rotation.y = -Math.PI / 2;
        model.rotation.z = -Math.PI / 2;

        // Center the model and align with Z-up
        const box = new THREE.Box3().setFromObject(model);
        const center = box.getCenter(new THREE.Vector3());

        // Offset everything so the bottom of the model is at Z=0
        model.position.x = -center.x;
        model.position.y = -center.y;
        model.position.z = -box.min.z;

        // Alignment handles Z-up orientation by default. 
        // Users can use Correction buttons for manual adjustments.

        // Calculate size for auto-scaling
        const size = box.getSize(new THREE.Vector3());

        // Parse the scene hierarchy
        this.parseHierarchy(model);

        // Identify movable components (X, Y, Z axes)
        this.identifyComponents(model);

        // ---- FREEZE EVERYTHING ELSE ----
        this.freezeNonMovableComponents(model);

        // ---- COLORIZE COMPONENTS ----
        this.colorizeComponents();

        // Hide loading
        const loadingElement = document.getElementById('loading');
        loadingElement.style.display = 'none';

        // Call completion callback
        if (this.onLoadComplete) {
            this.onLoadComplete({
                model: model,
                size: size,
                components: this.componentMap,
                tree: this.componentTree,
                animations: gltf.animations
            });
        }
    }

    parseHierarchy(object, level = 0, parentPath = '') {
        const path = parentPath ? `${parentPath}/${object.name}` : object.name;

        const nodeInfo = {
            name: object.name || 'Unnamed',
            type: object.type,
            path: path,
            level: level,
            object: object,
            children: []
        };

        // 🐛 DEBUG: Log hierarchy structure
        const indent = '  '.repeat(level);
        console.log(`${indent}📦 [${object.type}] "${object.name || 'Unnamed'}" (Level ${level})`);

        // Store in component map
        if (object.name) {
            this.componentMap.set(object.name, object);
        }

        // Recursively parse children
        if (object.children && object.children.length > 0) {
            object.children.forEach(child => {
                const childNode = this.parseHierarchy(child, level + 1, path);
                if (childNode) {
                    nodeInfo.children.push(childNode);
                }
            });
        }

        // Add to tree at root level
        if (level === 0) {
            this.componentTree.push(nodeInfo);
            console.log(`\n✅ Root node added to tree: "${nodeInfo.name}"\n`);
        }

        return nodeInfo;
    }

    identifyComponents(model) {
        console.log('\n🔍 Starting component identification...');

        // Try to identify X, Y, Z axis components by name patterns
        const components = {
            x: null,
            y: null,
            z: null
        };

        model.traverse((object) => {
            const name = object.name;
            const nameLower = name.toLowerCase();

            // 1. Slide_right_&_Left → UI Y slider (controls this component)
            if (name === 'Slide_right_&_Left' || (nameLower.includes('right') && nameLower.includes('left'))) {
                if (!components.x) {
                    components.x = object;  // Assign to Y
                    console.log(`  ✅ Found for UI Y-slider: "${name}" (Slide_right_&_Left)`);
                }
            }
            // 2. Main_push_&_pull → UI X slider (controls this component)
            if (name === 'Main_push_&_pull' || (nameLower.includes('push') && nameLower.includes('pull'))) {
                if (!components.y) {
                    components.y = object;  // Assign to X
                    console.log(`  ✅ Found for UI X-slider: "${name}" (Main_push_&_pull)`);
                }
            }
            // 3. Picker_up_&_down → UI Z axis
            if (name === 'Picker_up_&_down' || (nameLower.includes('picker') && (nameLower.includes('up') || nameLower.includes('down')))) {
                if (!components.z) {
                    components.z = object;
                    console.log(`  ✅ Found UI Z-axis: "${name}" (Picker_up_&_down)`);
                }
            }
        });

        // Store identified components
        this.identifiedComponents = components;

        // ✅ ONLY these can move
        this.movableComponents = new Set(
            Object.values(components).filter(Boolean)
        );

        console.log('\n📊 Component Identification Summary:');
        console.log('  UI X-slider (Left/Right) → ', components.x?.name || '❌ Not found');
        console.log('  UI Y-slider (Push/Pull)  → ', components.y?.name || '❌ Not found');
        console.log('  UI Z-slider (Up/Down)    → ', components.z?.name || '❌ Not found');
        console.log(`  Total movable parts: ${this.movableComponents.size}\n`);

        return components;
    }


    freezeNonMovableComponents(model) {
        // First, efficiently find all ancestors of movable components
        const movableAncestors = new Set();

        this.movableComponents.forEach(component => {
            let parent = component.parent;
            while (parent && parent !== model) {
                movableAncestors.add(parent);
                parent = parent.parent;
            }
        });

        model.traverse((object) => {
            // Skip root
            if (object === model || object.type === 'Scene') return;

            // 1. Is it a movable component?
            if (this.movableComponents.has(object)) return;

            // 2. Is it a parent/ancestor of a movable component?
            // If we freeze a parent, the child can't move relative to the world appropriately
            if (movableAncestors.has(object)) return;

            // Freeze transforms for everything else
            object.matrixAutoUpdate = false;
            object.updateMatrix();
        });

        console.log('Static non-axis components and non-ancestors frozen');
    }

    // ------------------------------------------------
    // COLORIZE COMPONENTS
    // ------------------------------------------------
    colorizeComponents() {
        // Define colors for each axis
        const colors = {
            x: 0xff0000,  // Pure bright red for X-axis
            y: 0x00ffff,  // Pure cyan for Y-axis
            z: 0x0080ff   // Bright blue for Z-axis
        };

        console.log('\n🎨 Starting component colorization...');

        // Apply colors to each identified component
        Object.keys(this.identifiedComponents).forEach(axis => {
            const component = this.identifiedComponents[axis];
            if (!component) return;

            const color = colors[axis];
            let meshCount = 0;

            // Traverse the component and apply color to all meshes
            component.traverse((object) => {
                if (object.isMesh) {
                    meshCount++;

                    // Handle both single materials and material arrays
                    const materials = Array.isArray(object.material)
                        ? object.material
                        : [object.material];

                    materials.forEach((mat, index) => {
                        if (mat) {
                            // Clone material to avoid affecting other objects
                            const newMat = mat.clone();
                            newMat.color.setHex(color);

                            // Add strong emissive glow for better visibility
                            if (newMat.emissive) {
                                newMat.emissive.setHex(color);
                                newMat.emissiveIntensity = 0.3;  // Increased for better visibility
                            }

                            // Apply the new material
                            if (Array.isArray(object.material)) {
                                object.material[index] = newMat;
                            } else {
                                object.material = newMat;
                            }
                        }
                    });
                }
            });

            console.log(`  🎨 Colorized ${axis.toUpperCase()}-axis: "${component.name}" (${meshCount} meshes)`);
        });

        console.log('Component colorization complete\n');
    }

    onProgress(progress) {
        if (progress.lengthComputable) {
            const percentComplete = (progress.loaded / progress.total) * 100;
            console.log(`Loading: ${percentComplete.toFixed(2)}%`);
        }
    }

    onError(error) {
        console.error('Error loading GLB model:', error);
        const loadingElement = document.getElementById('loading');
        loadingElement.innerHTML = '<p style="color: #ff4444;">Error loading GLB model. Please check the console.</p>';
        setTimeout(() => {
            loadingElement.style.display = 'none';
        }, 3000);
    }

    // Get all component names for UI
    getComponentNames() {
        return Array.from(this.componentMap.keys());
    }

    // Get component by name
    getComponent(name) {
        return this.componentMap.get(name);
    }

    // Generate HTML for component tree
    generateTreeHTML(tree = this.componentTree, level = 0) {
        let html = '';

        tree.forEach(node => {
            const hasChildren = node.children && node.children.length > 0;
            const expandIcon = hasChildren ? '▼' : '';
            const nodeIcon = this.getNodeIcon(node);

            html += `
                <div class="tree-item" data-level="${level}">
                    <div class="tree-node" data-component="${node.name}" data-path="${node.path}">
                        <span class="expand-icon">${expandIcon}</span>
                        <span class="node-icon">${nodeIcon}</span>
                        <span class="node-label">${node.name}</span>
                    </div>`;

            if (hasChildren) {
                html += '<div class="tree-children">';
                html += this.generateTreeHTML(node.children, level + 1);
                html += '</div>';
            }

            html += '</div>';
        });

        return html;
    }

    getNodeIcon(node) {
        const name = node.name.toLowerCase();
        const type = node.type;

        // Icon based on type and name
        if (type === 'Mesh') return '🔷';
        if (type === 'Group' || type === 'Object3D') {
            if (name.includes('x')) return '🔴';
            if (name.includes('y')) return '🟢';
            if (name.includes('z')) return '🔵';
            return '📦';
        }
        if (type === 'Scene') return '🌐';

        return '📁';
    }
}

// Export for use in other scripts
window.GLBLoader = GLBLoader;
