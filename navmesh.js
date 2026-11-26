/* global AFRAME */

/**
 * Builds a walkable NavMesh (Three.js BufferGeometry) from a ground plane
 * and a set of obstacle boxes, and exposes A* pathfinding over the grid.
 */
AFRAME.registerComponent('navmesh-builder', {
  schema: {
    ground: { type: 'selector' },
    obstacles: { type: 'string', default: '.obstacle' },
    cellSize: { type: 'number', default: 0.5 },
    agentRadius: { type: 'number', default: 0.4 },
    yOffset: { type: 'number', default: 0.03 }
  },

  init() {
    this.walkableGrid = [];
    this.cols = 0;
    this.rows = 0;
    this.bounds = null;

    // Wait for scene & geometry to be ready
    this.el.sceneEl.addEventListener('loaded', () => {
      this.buildNavMesh();
    });
  },

  buildNavMesh() {
    const THREE = AFRAME.THREE;
    const { ground, obstacles, cellSize, agentRadius, yOffset } = this.data;
    if (!ground) return;

    ground.object3D.updateMatrixWorld(true);
    const groundBox = new THREE.Box3().setFromObject(ground.object3D);
    this.bounds = groundBox;

    // Collect expanded bounding boxes for all obstacles
    const obstacleEls = Array.from(this.el.sceneEl.querySelectorAll(obstacles));
    const obstacleBoxes = obstacleEls.map((el) => {
      el.object3D.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(el.object3D);
      box.expandByScalar(agentRadius);
      return box;
    });

    const width = groundBox.max.x - groundBox.min.x;
    const depth = groundBox.max.z - groundBox.min.z;
    this.cols = Math.floor(width / cellSize);
    this.rows = Math.floor(depth / cellSize);

    const raycaster = new THREE.Raycaster();
    const down = new THREE.Vector3(0, -1, 0);
    const rayOriginY = groundBox.max.y + 10;

    const positions = [];
    const indices = [];
    let vertexCount = 0;

    this.walkableGrid = Array.from({ length: this.rows }, () => Array(this.cols).fill(false));

    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        const x0 = groundBox.min.x + c * cellSize;
        const z0 = groundBox.min.z + r * cellSize;
        const cx = x0 + cellSize * 0.5;
        const cz = z0 + cellSize * 0.5;

        const blocked = obstacleBoxes.some(
          (b) => cx >= b.min.x && cx <= b.max.x && cz >= b.min.z && cz <= b.max.z
        );
        if (blocked) continue;

        // Raycast down onto any ground surface (flat, sloped, or 3D mesh)
        raycaster.set(new THREE.Vector3(cx, rayOriginY, cz), down);
        const hits = raycaster.intersectObject(ground.object3D, true);
        if (hits.length === 0) continue;

        this.walkableGrid[r][c] = true;
        const x1 = x0 + cellSize;
        const z1 = z0 + cellSize;
        const y = hits[0].point.y + yOffset;

        positions.push(
          x0, y, z0,
          x1, y, z0,
          x1, y, z1,
          x0, y, z1
        );

        indices.push(
          vertexCount, vertexCount + 2, vertexCount + 1,
          vertexCount, vertexCount + 3, vertexCount + 2
        );
        vertexCount += 4;
      }
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();

    const material = new THREE.MeshBasicMaterial({
      color: 0x22c55e,
      wireframe: true,
      transparent: true,
      opacity: 0.65,
      side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geometry, material);
    this.el.setObject3D('mesh', mesh);
  },

  worldToGrid(pos) {
    const { cellSize } = this.data;
    const c = Math.floor((pos.x - this.bounds.min.x) / cellSize);
    const r = Math.floor((pos.z - this.bounds.min.z) / cellSize);
    return {
      r: Math.max(0, Math.min(this.rows - 1, r)),
      c: Math.max(0, Math.min(this.cols - 1, c))
    };
  },

  isWalkable(x, z) {
    if (!this.bounds || this.walkableGrid.length === 0) return true;
    const { cellSize } = this.data;
    const c = Math.floor((x - this.bounds.min.x) / cellSize);
    const r = Math.floor((z - this.bounds.min.z) / cellSize);
    if (r < 0 || r >= this.rows || c < 0 || c >= this.cols) return false;
    return Boolean(this.walkableGrid[r][c]);
  },

  getRandomWalkablePosition() {
    if (!this.bounds || this.walkableGrid.length === 0) {
      return new AFRAME.THREE.Vector3(0, 0, 0);
    }
    const walkable = [];
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.walkableGrid[r][c]) walkable.push([r, c]);
      }
    }
    if (walkable.length === 0) return new AFRAME.THREE.Vector3(0, 0, 0);
    const [r, c] = walkable[Math.floor(Math.random() * walkable.length)];
    return this.gridToWorld(r, c);
  },

  gridToWorld(r, c) {
    const THREE = AFRAME.THREE;
    const { cellSize } = this.data;
    return new THREE.Vector3(
      this.bounds.min.x + (c + 0.5) * cellSize,
      0,
      this.bounds.min.z + (r + 0.5) * cellSize
    );
  },

  findPath(startPos, endPos) {
    if (!this.bounds || this.walkableGrid.length === 0) return [];

    const start = this.worldToGrid(startPos);
    const goal = this.worldToGrid(endPos);

    if (!this.walkableGrid[goal.r]?.[goal.c]) return [];

    const key = (r, c) => `${r},${c}`;
    const open = [{ r: start.r, c: start.c, g: 0, f: 0 }];
    const cameFrom = new Map();
    const gScore = new Map([[key(start.r, start.c), 0]]);
    const closed = new Set();

    const dirs = [
      [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
      [1, 1, 1.414], [1, -1, 1.414], [-1, 1, 1.414], [-1, -1, 1.414]
    ];

    while (open.length > 0) {
      open.sort((a, b) => a.f - b.f);
      const curr = open.shift();
      const currKey = key(curr.r, curr.c);

      if (curr.r === goal.r && curr.c === goal.c) {
        const path = [];
        let k = currKey;
        while (k) {
          const [r, c] = k.split(',').map(Number);
          path.unshift(this.gridToWorld(r, c));
          k = cameFrom.get(k);
        }
        return path;
      }

      closed.add(currKey);

      for (const [dr, dc, cost] of dirs) {
        const nr = curr.r + dr;
        const nc = curr.c + dc;
        if (nr < 0 || nr >= this.rows || nc < 0 || nc >= this.cols) continue;
        if (!this.walkableGrid[nr][nc]) continue;

        const nKey = key(nr, nc);
        if (closed.has(nKey)) continue;

        const tentativeG = curr.g + cost;
        if (tentativeG < (gScore.get(nKey) ?? Infinity)) {
          cameFrom.set(nKey, currKey);
          gScore.set(nKey, tentativeG);
          const h = Math.hypot(goal.r - nr, goal.c - nc);
          open.push({ r: nr, c: nc, g: tentativeG, f: tentativeG + h });
        }
      }
    }

    return [];
  }
});

/**
 * Moves an entity along the path computed by navmesh-builder when clicking .walkable.
 */
AFRAME.registerComponent('nav-agent', {
  schema: {
    navmesh: { type: 'selector' },
    speed: { type: 'number', default: 4 }
  },

  init() {
    this.path = [];
    this.el.sceneEl.addEventListener('click', (evt) => {
      const intersection = evt.detail?.intersection;
      if (!intersection || !this.data.navmesh) return;

      const nav = this.data.navmesh.components['navmesh-builder'];
      if (!nav) return;

      const newPath = nav.findPath(this.el.object3D.position, intersection.point);
      if (newPath.length > 0) {
        this.path = newPath;
      }
    });
  },

  tick(time, delta) {
    if (this.path.length === 0) return;

    const dt = Math.min(delta / 1000, 0.1);
    const pos = this.el.object3D.position;
    const target = this.path[0];

    const dx = target.x - pos.x;
    const dz = target.z - pos.z;
    const dist = Math.hypot(dx, dz);

    if (dist < 0.1) {
      this.path.shift();
      return;
    }

    const step = Math.min(dist, this.data.speed * dt);
    pos.x += (dx / dist) * step;
    pos.z += (dz / dist) * step;
  }
});

/**
 * Constrains a POV camera/player entity to walkable cells on the navmesh,
 * allowing smooth wall-sliding along obstacle edges.
 */
AFRAME.registerComponent('navmesh-constrain', {
  schema: {
    navmesh: { type: 'selector' }
  },

  init() {
    this.lastValidPos = new AFRAME.THREE.Vector3();
    this.initialized = false;
  },

  tick() {
    const navEl = this.data.navmesh;
    const nav = navEl?.components['navmesh-builder'];
    if (!nav || !nav.bounds) return;

    const pos = this.el.object3D.position;

    if (!this.initialized) {
      this.lastValidPos.copy(pos);
      this.initialized = true;
      return;
    }

    if (nav.isWalkable(pos.x, pos.z)) {
      this.lastValidPos.copy(pos);
      return;
    }

    // Slide along X or Z if only one axis is blocked
    if (nav.isWalkable(pos.x, this.lastValidPos.z)) {
      pos.z = this.lastValidPos.z;
      this.lastValidPos.copy(pos);
    } else if (nav.isWalkable(this.lastValidPos.x, pos.z)) {
      pos.x = this.lastValidPos.x;
      this.lastValidPos.copy(pos);
    } else {
      pos.x = this.lastValidPos.x;
      pos.z = this.lastValidPos.z;
    }
  }
});

/**
 * Enemy bot that chases the player using the navmesh and respawns when shot.
 */
AFRAME.registerComponent('enemy-bot', {
  schema: {
    navmesh: { type: 'selector' },
    target: { type: 'selector' },
    speed: { type: 'number', default: 2.2 }
  },

  init() {
    this.path = [];
    this.repathTimer = 0;
    this.alive = true;

    this.el.sceneEl.addEventListener('loaded', () => {
      this.respawn();
    });
  },

  respawn() {
    const nav = this.data.navmesh?.components['navmesh-builder'];
    if (!nav) return;
    const pos = nav.getRandomWalkablePosition();
    this.el.object3D.position.set(pos.x, 0.75, pos.z);
    this.el.setAttribute('visible', true);
    this.path = [];
    this.alive = true;
  },

  hit() {
    if (!this.alive) return;
    this.alive = false;
    this.el.setAttribute('visible', false);
    this.el.sceneEl.emit('enemy-killed');
    setTimeout(() => this.respawn(), 1200);
  },

  tick(time, delta) {
    if (!this.alive || !this.data.navmesh || !this.data.target) return;
    const nav = this.data.navmesh.components['navmesh-builder'];
    if (!nav || !nav.bounds) return;

    const dt = Math.min(delta / 1000, 0.1);
    this.repathTimer += dt;

    if (this.repathTimer >= 0.6) {
      this.repathTimer = 0;
      const targetPos = this.data.target.object3D.position;
      this.path = nav.findPath(this.el.object3D.position, targetPos);
    }

    if (this.path.length === 0) return;

    const pos = this.el.object3D.position;
    const next = this.path[0];
    const dx = next.x - pos.x;
    const dz = next.z - pos.z;
    const dist = Math.hypot(dx, dz);

    if (dist < 0.15) {
      this.path.shift();
      return;
    }

    const step = Math.min(dist, this.data.speed * dt);
    pos.x += (dx / dist) * step;
    pos.z += (dz / dist) * step;
  }
});

/**
 * First-person shooter component attached to the camera.
 * Fires a raycast + visible laser tracer on click or Spacebar.
 */
AFRAME.registerComponent('player-shooter', {
  init() {
    const THREE = AFRAME.THREE;
    this.raycaster = new THREE.Raycaster();
    this.center = new THREE.Vector2(0, 0);

    const shoot = () => this.shoot();
    window.addEventListener('mousedown', (e) => {
      if (e.target.tagName === 'BUTTON') return;
      shoot();
    });
    window.addEventListener('keydown', (e) => {
      if (e.code === 'Space') shoot();
    });
  },

  shoot() {
    const THREE = AFRAME.THREE;
    const cam = this.el.getObject3D('camera');
    if (!cam) return;

    this.raycaster.setFromCamera(this.center, cam);

    const enemyEls = Array.from(this.el.sceneEl.querySelectorAll('.enemy'));
    const enemyMeshes = [];
    enemyEls.forEach((el) => {
      if (el.getAttribute('visible') !== false) {
        el.object3D.traverse((obj) => {
          if (obj.isMesh) {
            obj.userData.enemyEl = el;
            enemyMeshes.push(obj);
          }
        });
      }
    });

    const hits = this.raycaster.intersectObjects(enemyMeshes, false);
    const start = new THREE.Vector3();
    cam.getWorldPosition(start);
    start.y -= 0.15;

    const dir = new THREE.Vector3();
    cam.getWorldDirection(dir);
    const end = hits.length > 0
      ? hits[0].point
      : start.clone().add(dir.multiplyScalar(25));

    this.spawnTracer(start, end);

    if (hits.length > 0) {
      const enemyEl = hits[0].object.userData.enemyEl;
      enemyEl?.components['enemy-bot']?.hit();
    }
  },

  spawnTracer(start, end) {
    const THREE = AFRAME.THREE;
    const geom = new THREE.BufferGeometry().setFromPoints([start, end]);
    const mat = new THREE.LineBasicMaterial({ color: 0x38bdf8 });
    const line = new THREE.Line(geom, mat);
    this.el.sceneEl.object3D.add(line);
    setTimeout(() => {
      this.el.sceneEl.object3D.remove(line);
      geom.dispose();
      mat.dispose();
    }, 80);
  }
});

