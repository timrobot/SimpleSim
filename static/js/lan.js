import * as THREE from 'three';

class WSConnection {
  constructor(port, width, height, near, far, peerid) {
    port = port || 9999;
    this.peerid = peerid;

    // Offscreen elements (cameras)
    this.colorURL;
    this.depthURL;

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


    // data hooks for ws
    this.observation = [];
    this.reward = 0;
    this.terminal = false;
    this.requires_render = false;
    // this.cmd = null;

    this.onreset = null;
    this.onstep = null;

    this.ws = new WebSocket(`ws://127.0.0.1:${port}`);
    this.ws_connected = false;
    this.ws.onopen = () => {
      console.log('WebSocket connection established');
      this.ws_connected = true;
    };

    this.ws.onmessage = (event) => {
      const cmd = JSON.parse(event.data);
      if (cmd.api === "reset" && this.onreset) {
        this.onreset();
        this._update_callback();
      } else if (cmd.api === "act" && this.onstep) {
        this.onstep(cmd.action);
        this._update_callback();
      } else if (cmd.api === "render") {
        this.requires_render = cmd.value;
        this.ws.send(JSON.stringify("success"));
      } else if (cmd.api === "multiplayerConnect") {
        // do something here
      }
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

    // this._update_callback();
  }

  _update_callback() {
    if (this.ws_connected) {
      if (this.requires_render) {
        if (this.colorURL && this.depthURL) {
          const color = this.colorURL;
          const depth = this.depthURL;
          this.colorURL = null;
          this.depthURL = null;
          const info = JSON.stringify({
            "obs": this.observation,
            "rew": this.reward,
            "term": this.terminal
          });
          this.ws.send(info + '$' + color + ',' + depth);
        } else {
          setTimeout(this._update_callback.bind(this), 2);
        }
      } else {
        this.colorURL = null;
        this.depthURL = null;
        const info = JSON.stringify({
          "obs": this.observation,
          "rew": this.reward,
          "term": this.terminal
        });
        this.ws.send(info + '$');
      }
    }
  }

  setObservationReward(obs, rew, term) {
    this.observation = obs;
    this.reward = rew;
    this.terminal = term || false;
  }

  render(scene, offscreenCamera) {
    if (!this.requires_render) return;

    // Render the scene to offscreen targets
    this.colorRenderer.render(scene, offscreenCamera);

    // render scene into target
    this.depthRenderer.setRenderTarget(this.target);
    this.depthRenderer.render(scene, offscreenCamera);

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
      };
      colorreader.readAsDataURL(blob);
    });
    this.depthCanvas.convertToBlob(options).then((blob) => {
      let depthreader = new FileReader();
      depthreader.onload = () => {
        this.depthURL = depthreader.result;
      };
      depthreader.readAsDataURL(blob);
    });

  }
};

export default WSConnection;