// Animator.js - Smooth time-based animation system
class Animator {
    constructor() {
        this.animations = {
            x: null,
            y: null,
            z: null
        };
        this.duration = 5000; // 5 seconds in milliseconds
    }

    // Easing function for smooth transitions (ease-in-out)
    easeInOutCubic(t) {
        return t < 0.5
            ? 4 * t * t * t
            : 1 - Math.pow(-2 * t + 2, 3) / 2;
    }

    // Start animation for a specific axis
    animateTo(axis, component, targetPosition, onUpdate, property = null) {
        if (!component) {
            console.warn(`Component for ${axis} axis not found`);
            return;
        }

        // Use specifically provided property or default to the axis name
        const prop = property || axis;

        // Cancel existing animation for this axis
        if (this.animations[axis]) {
            cancelAnimationFrame(this.animations[axis].frameId);
        }

        const startPosition = component.position[prop];
        const startTime = performance.now();

        const animate = (currentTime) => {
            const elapsed = currentTime - startTime;
            const progress = Math.min(elapsed / this.duration, 1);
            const easedProgress = this.easeInOutCubic(progress);

            // Interpolate position
            const currentPosition = startPosition + (targetPosition - startPosition) * easedProgress;
            component.position[prop] = currentPosition;

            // Call update callback if provided
            if (onUpdate) {
                onUpdate(currentPosition);
            }

            // Continue animation if not complete
            if (progress < 1) {
                this.animations[axis].frameId = requestAnimationFrame(animate);
            } else {
                this.animations[axis] = null;
            }
        };

        // Store animation state
        this.animations[axis] = {
            frameId: requestAnimationFrame(animate),
            startPosition,
            targetPosition,
            startTime
        };
    }

    // Check if any animation is currently running
    isAnimating() {
        return Object.values(this.animations).some(anim => anim !== null);
    }

    // Get current animation progress for an axis (0-1)
    getProgress(axis) {
        const anim = this.animations[axis];
        if (!anim) return 1;

        const elapsed = performance.now() - anim.startTime;
        return Math.min(elapsed / this.duration, 1);
    }

    // Cancel all animations
    cancelAll() {
        Object.keys(this.animations).forEach(axis => {
            if (this.animations[axis]) {
                cancelAnimationFrame(this.animations[axis].frameId);
                this.animations[axis] = null;
            }
        });
    }

    // Cancel animation for specific axis
    cancel(axis) {
        if (this.animations[axis]) {
            cancelAnimationFrame(this.animations[axis].frameId);
            this.animations[axis] = null;
        }
    }
}

// Export for use in other scripts
window.Animator = Animator;
