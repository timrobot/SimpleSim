import * as THREE from 'three';

class WSConnection {
  constructor(port, width, height, near, far, scene, offscreenCamera) {
    port = port || 9999;

    // Offscreen elements (cameras)
    this.colorURL;
    this.depthURL;

    this.offscreenCamera = offscreenCamera;
    this.scene = scene;

    this.colorCanvas = new OffscreenCanvas(width, height);
    const colorContext = this.colorCanvas.getContext('webgl2', { antialias: true, alpha: false });
    this.colorRenderer = new THREE.WebGLRenderer({canvas: this.colorCanvas, context: colorContext, antialias: true});
    this.colorRenderer.shadowMap.enabled = true;
    this.colorRenderer.setPixelRatio(1.0);
    this.colorRenderer.setClearColor(0x80c0e0);

    this.depthCanvas = new OffscreenCanvas(width, height);
    const depthContext = this.depthCanvas.getContext('webgl2', { alpha: false });
    this.depthRenderer = new THREE.WebGLRenderer({canvas: this.depthCanvas, context: depthContext});
    this.depthRenderer.setPixelRatio(1.0);
    this.depthRenderer.setClearColor(0x80c0e0);

    // Set up depth pre-processing
    this.target; // render target for rgbd

    const params = {
      format: THREE.DepthFormat,
      type: THREE.UnsignedShortType,
      samples: 0,
    };

    const formats = { DepthFormat: THREE.DepthFormat, DepthStencilFormat: THREE.DepthStencilFormat };
    const types = { UnsignedShortType: THREE.UnsignedShortType, UnsignedIntType: THREE.UnsignedIntType, FloatType: THREE.FloatType };

    if ( this.target ) this.target.dispose();

    const format = parseInt( params.format );
    const type = parseInt( params.type );
    const samples = parseInt( params.samples );

    const dpr = this.depthRenderer.getPixelRatio();
    this.target = new THREE.WebGLRenderTarget( window.innerWidth * dpr, window.innerHeight * dpr );
    this.target.texture.minFilter = THREE.NearestFilter;
    this.target.texture.magFilter = THREE.NearestFilter;
    this.target.stencilBuffer = ( format === THREE.DepthStencilFormat ) ? true : false;
    this.target.samples = samples;

    this.target.depthTexture = new THREE.DepthTexture();
    this.target.depthTexture.format = format;
    this.target.depthTexture.type = type;


    // Setup post processing stage
    this.postCamera = new THREE.OrthographicCamera( - 1, 1, 1, - 1, 0, 1 );
    this.postMaterial = new THREE.ShaderMaterial( {
      vertexShader: document.querySelector( '#post-vert' ).textContent.trim(),
      fragmentShader: document.querySelector( '#post-frag' ).textContent.trim(),
      uniforms: {
        cameraNear: { value: near },
        cameraFar: { value: far },
        tDiffuse: { value: null },
        tDepth: { value: null }
      }
    } );
    const postPlane = new THREE.PlaneGeometry( 2, 2 );
    const postQuad = new THREE.Mesh( postPlane, this.postMaterial );
    this.postScene = new THREE.Scene();
    this.postScene.add( postQuad );

    this.bodies = {};

    this.ws = new WebSocket(`ws://127.0.0.1:${port}`);
    this.ws_connected = false;
    this.ws.onopen = () => {
      console.log('WebSocket connection established');
      this.ws_connected = true;
    };

    this.ws.onmessage = (event) => {
      const state = JSON.parse(event.data);
      state.bodies.forEach((body) => {
        if (this.bodies.hasOwnProperty(body.name)) {
          const b = body.name === 'camera' ? this.offscreenCamera : this.bodies[body.name];
          b.position.set(...body.position);
          b.quaternion.set(...body.quaternion);
        } else {
          console.log(`Cannot find body ${body.name}`);
        }
      });
      this.render();
    };

    this.ws.onclose = () => {
      console.log('WebSocket connection closed, closing window after 0.5 seconds.');
      this.ws_connected = false;
      setTimeout(function () {
        window.close();
      }, 500);
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
      this.ws_connected = false;
    };
  }

  send_render() {
    if (this.ws_connected) {
      if (this.colorURL && this.depthURL) {
        const color = this.colorURL;
        const depth = this.depthURL;
        this.colorURL = null;
        this.depthURL = null;
        // send the rendered frames over
        this.ws.send(color + ',' + depth);
      }
    }
  }

  render() {
    if (!this.offscreenCamera || !this.scene) {
      console.error('Offscreen camera or scene not set, cannot render.');
      return;
    }

    // Render the scene to offscreen targets
    this.colorRenderer.render(this.scene, this.offscreenCamera);

    // render scene into target
    this.depthRenderer.setRenderTarget(this.target);
    this.depthRenderer.render(this.scene, this.offscreenCamera);

    // render post FX
    this.postMaterial.uniforms.tDiffuse.value = this.target.texture;
    this.postMaterial.uniforms.tDepth.value = this.target.depthTexture;

    this.depthRenderer.setRenderTarget(null);
    this.depthRenderer.render(this.postScene, this.postCamera);

    const options = {
      type: 'image/webp', // Specify the desired format
      quality: 1.0, // Set the image quality (optional, 0.0 to 1.0)
    };

    this.colorCanvas.convertToBlob(options).then((blob) => {
      let colorreader = new FileReader();
      colorreader.onload = () => {
        this.colorURL = colorreader.result;
        if (this.depthURL) {
          this.send_render(); // Send the data if both URLs are ready
        }
      };
      colorreader.readAsDataURL(blob);
    });
    this.depthCanvas.convertToBlob(options).then((blob) => {
      let depthreader = new FileReader();
      depthreader.onload = () => {
        this.depthURL = depthreader.result;
        if (this.colorURL) {
          this.send_render(); // Send the data if both URLs are ready
        }
      };
      depthreader.readAsDataURL(blob);
    });
  }

  addBody(body) {
    // this function acts as a proxy for a socket connection
    this.bodies[body.name] = body;
  }
  removeBody(body) {
    // this function acts as a proxy for a socket connection
    if (this.bodies.hasOwnProperty(body.name)) {
      delete this.bodies[body.name];
    }
  }
};

export default WSConnection;