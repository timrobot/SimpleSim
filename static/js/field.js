import * as THREE from 'three';

const textureLoader = new THREE.TextureLoader();
textureLoader.crossOrigin = 'anonymous';

function loadTexture(path) {
  const filepath = `./static/assets/${path}`;
  const texture = textureLoader.load(filepath);
  texture.colorSpace = THREE.SRGBColorSpace;
  // texture.minFilter = THREE.NearestFilter; // we dont want blocky
  // texture.magFilter = THREE.NearestFilter;
  return texture;
}

class Field extends THREE.Group {
  constructor() {
    super();
    this.tag16h5 = {};
  }

  initMeshes(physx) {
    const in2m = (x) => x * 0.0254;

    const groundGeometry = new THREE.BoxGeometry(50, 1, 50);
    const groundMaterial = new THREE.MeshLambertMaterial({color: 0xdacfa3});
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.position.set(0, -0.5, 0);
    ground.castShadow = true;
    ground.receiveShadow = true;
    this.add(ground);
    physx.add(ground, {collideGroup: 1, collideWith: 0xFF});

    const walls = [];
    const wallGeometry = new THREE.BoxGeometry(in2m(145), in2m(13.5), in2m(1));
    const wallMaterial = new THREE.MeshLambertMaterial({color: 0xd6d9cc});
    for (let i = 0; i < 4; i++) {
      walls.push(new THREE.Mesh(wallGeometry, wallMaterial));
    }
    walls[0].position.set( 0, in2m(6.75), in2m(73));
    walls[1].position.set( in2m(73), in2m(6.75), 0);
    walls[1].rotateY(Math.PI / 2);
    walls[2].position.set( 0, in2m(6.75),-in2m(73));
    walls[3].position.set(-in2m(73), in2m(6.75), 0);
    walls[3].rotateY(Math.PI / 2);
    for (const wall of walls) {
      wall.castShadow = true;
      wall.receiveShadow = true;
      this.add(wall);
      physx.add(wall, {collideGroup: 1, collideWith: 0xFF});
    }

    const columnGeometry = new THREE.BoxGeometry(in2m(4), in2m(13.5), in2m(4));
    const columnMaterial = new THREE.MeshLambertMaterial({color: 0xd6d9cc});
    const column = new THREE.Mesh(columnGeometry, columnMaterial);
    column.position.set(0, in2m(6.75), 0);
    column.castShadow = true;
    column.receiveShadow = true;
    this.add(column);
    physx.add(column, {collideGroup: 1, collideWith: 0xFF});

    const black_material = new THREE.MeshLambertMaterial({color: 0x080808});
    for (let i = 0; i < 28; i++) {
      const texture = loadTexture(`tag16h5_000${i < 10 ? '0': ''}${i}.png`);
      const tag_material = new THREE.MeshLambertMaterial({map: texture});
      const frame_material = [black_material, black_material, tag_material, black_material, black_material, black_material];
      const frame_geometry = new THREE.BoxGeometry(in2m(4), in2m(0.5), in2m(4));
      const frame_mesh = new THREE.Mesh(frame_geometry, frame_material);
      this.tag16h5[i] = frame_mesh;
    }

    for (let i = 0; i < 24; i++) {
      this.tag16h5[i].rotateX(Math.PI / 2);
      if (i < 6) {
        this.tag16h5[i].rotateZ(0);
        this.tag16h5[i].position.set(in2m(60 - 24 * i), in2m(12), in2m(-72));
      } else if (6 <= i && i < 12) {
        this.tag16h5[i].rotateZ(-Math.PI / 2);
        this.tag16h5[i].position.set(in2m(-72), in2m(12), in2m(-60 + 24 * (i - 6)));
      } else if (12 <= i && i < 18) {
        this.tag16h5[i].rotateZ(Math.PI);
        this.tag16h5[i].position.set(in2m(-60 + 24 * (i - 12)), in2m(12), in2m(72));
      } else if (18 <= i && i < 24) {
        this.tag16h5[i].rotateZ(Math.PI / 2);
        this.tag16h5[i].position.set(in2m(72), in2m(12), in2m(60 - 24 * (i - 18)));
      }
      this.add(this.tag16h5[i]);
    }

    this.tag16h5[24].rotateX(Math.PI / 2);
    this.tag16h5[24].rotateZ(Math.PI);
    this.tag16h5[24].position.set(0, in2m(12), in2m(-2.25));
    this.add(this.tag16h5[24]);
    this.tag16h5[25].rotateX(Math.PI / 2);
    this.tag16h5[25].rotateZ(Math.PI / 2);
    this.tag16h5[25].position.set(in2m(-2.25), in2m(12), 0);
    this.add(this.tag16h5[25]);
    this.tag16h5[26].rotateX(Math.PI / 2);
    this.tag16h5[26].position.set(0, in2m(12), in2m(2.25));
    this.add(this.tag16h5[26]);
    this.tag16h5[27].rotateX(Math.PI / 2);
    this.tag16h5[27].rotateZ(-Math.PI / 2);
    this.tag16h5[27].position.set(in2m(2.25), in2m(12), 0);
    this.add(this.tag16h5[27]);
  }
};

export { Field };