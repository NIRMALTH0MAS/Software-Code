// UI Manager - Handles sidebar toggles, component tree, and file loading
class UIManager {
    constructor() {
        this.leftSidebar = document.getElementById('leftSidebar');
        this.rightSidebar = document.getElementById('rightSidebar');
        this.toggleLeftBtn = document.getElementById('toggleLeftSidebar');
        this.toggleRightBtn = document.getElementById('toggleRightSidebar');
        this.closeLeftBtn = document.getElementById('closeLeftSidebar');
        this.closeRightBtn = document.getElementById('closeRightSidebar');
        this.fileInput = document.getElementById('stlFileInput');
        this.currentFileName = document.getElementById('currentFileName');

        this.initializeEventListeners();
        this.initializeComponentTree();
    }

    initializeEventListeners() {
        const canvasContainer = document.querySelector('.canvas-container');

        // Helper to trigger window resize (needed for Three.js renderer update)
        const triggerResize = () => {
            setTimeout(() => {
                window.dispatchEvent(new Event('resize'));
            }, 310); // Match transition duration (300ms) + small buffer
        };

        // Left sidebar toggle
        this.closeLeftBtn.addEventListener('click', () => {
            this.leftSidebar.classList.add('hidden');
            this.toggleLeftBtn.classList.add('visible');
            canvasContainer.classList.add('left-closed');
            triggerResize();
        });

        this.toggleLeftBtn.addEventListener('click', () => {
            this.leftSidebar.classList.remove('hidden');
            this.toggleLeftBtn.classList.remove('visible');
            canvasContainer.classList.remove('left-closed');
            triggerResize();
        });

        // Right sidebar toggle
        this.closeRightBtn.addEventListener('click', () => {
            this.rightSidebar.classList.add('hidden');
            this.toggleRightBtn.classList.add('visible');
            canvasContainer.classList.add('right-closed');
            triggerResize();
        });

        this.toggleRightBtn.addEventListener('click', () => {
            this.rightSidebar.classList.remove('hidden');
            this.toggleRightBtn.classList.remove('visible');
            canvasContainer.classList.remove('right-closed');
            triggerResize();
        });

        // File input
        this.fileInput.addEventListener('change', (e) => {
            this.handleFileUpload(e);
        });
    }

    initializeComponentTree() {
        const treeNodes = document.querySelectorAll('.tree-node');

        treeNodes.forEach(node => {
            node.addEventListener('click', (e) => {
                e.stopPropagation();

                // Handle expand/collapse
                const expandIcon = node.querySelector('.expand-icon');
                if (expandIcon && expandIcon.textContent) {
                    node.classList.toggle('collapsed');
                }

                // Handle selection
                treeNodes.forEach(n => n.classList.remove('selected'));
                node.classList.add('selected');

                // Get component name and highlight in 3D scene
                const componentName = node.dataset.component;
                if (componentName && window.highlightComponent) {
                    window.highlightComponent(componentName);
                }
            });
        });
    }

    handleFileUpload(event) {
        const file = event.target.files[0];
        if (!file) return;

        const fileName = file.name.toLowerCase();
        const isGLB = fileName.endsWith('.glb') || fileName.endsWith('.gltf');
        const isSTL = fileName.endsWith('.stl');

        if (!isGLB && !isSTL) {
            alert('Please select a valid GLB, GLTF, or STL file');
            return;
        }

        // Update filename display
        this.currentFileName.textContent = file.name;

        // Read and load the file
        const reader = new FileReader();
        reader.onload = (e) => {
            const arrayBuffer = e.target.result;

            if (isGLB) {
                // GLB format - full component interaction
                if (window.loadGLBFromBuffer) {
                    window.loadGLBFromBuffer(arrayBuffer, file.name);
                }
            } else if (isSTL) {
                // STL format - visualization only
                if (window.loadSTLFromBuffer) {
                    window.loadSTLFromBuffer(arrayBuffer, file.name);
                }
            }
        };
        reader.readAsArrayBuffer(file);
    }

    updateComponentTree(components) {
        // This can be called to dynamically update the tree
        // when a new model is loaded
        console.log('Component tree updated for:', components);
    }
}

// Export for use in other scripts
window.UIManager = UIManager;
