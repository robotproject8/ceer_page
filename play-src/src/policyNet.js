// policyNet.js — pure-JS forward pass of the CEER policy network, replacing
// onnxruntime-web (drops a 26 MB wasm runtime + its loader complexity). The
// architecture is fixed and was read directly from the ONNX graph:
//
//   every block:  y = Mish(LayerNorm(Linear(x)))        Mish(x)=x*tanh(softplus(x))
//   LayerNorm eps = 1e-5, axis = -1;  Linear = x·Wᵀ + b (onnx Gemm transB=1)
//
//   adapt(obs257)       : L257→512 ·LN·Mish · L512→256 ·LN·Mish · L256→256
//   adapt_joint(obs257) : L257→512 ·LN·Mish · L512→256 ·LN·Mish · L256→145
//   actor(cat[obs,adapt,adapt_joint]=658):
//                         L658→512 ·LN·Mish · L512→512 ·LN·Mish · L512→256 ·LN·Mish · L256→29
//
// Validated against golden fixtures (control_test.mjs) to <1e-4 vs PyTorch/ONNX.

function softplus(x) {
  // numerically stable: log1p(exp(x)) = max(x,0) + log1p(exp(-|x|))
  const ax = Math.abs(x);
  return Math.max(x, 0) + Math.log1p(Math.exp(-ax));
}
function mishInPlace(v) {
  for (let i = 0; i < v.length; i++) v[i] = v[i] * Math.tanh(softplus(v[i]));
  return v;
}

export class PolicyNet {
  /**
   * @param {ArrayBuffer} weightsBuf  policy_weights.bin (float32 LE)
   * @param {object} arch             policy_arch.json ({eps, tensors:{name:{offset,shape}}})
   */
  constructor(weightsBuf, arch) {
    this.eps = arch.eps ?? 1e-5;
    this.f32 = new Float32Array(weightsBuf);
    this.t = {}; // name -> { data: Float32Array view, shape }
    for (const [name, { offset, shape }] of Object.entries(arch.tensors)) {
      const n = shape.reduce((a, b) => a * b, 1);
      this.t[name] = { data: this.f32.subarray(offset, offset + n), shape };
    }
    // scratch reused across calls (single-threaded worker use)
    this._input658 = new Float32Array(658);
  }

  // y[out] = x·Wᵀ + b, W shape [out,in] row-major.
  _linear(x, wName, bName) {
    const W = this.t[wName].data;
    const [out, inn] = this.t[wName].shape;
    const b = this.t[bName].data;
    const y = new Float32Array(out);
    for (let o = 0; o < out; o++) {
      let s = b[o];
      const base = o * inn;
      for (let i = 0; i < inn; i++) s += x[i] * W[base + i];
      y[o] = s;
    }
    return y;
  }

  // LayerNorm over the whole vector (axis -1), in place.
  _layerNormInPlace(x, gName, bName) {
    const g = this.t[gName].data;
    const b = this.t[bName].data;
    const n = x.length;
    let mean = 0;
    for (let i = 0; i < n; i++) mean += x[i];
    mean /= n;
    let varc = 0;
    for (let i = 0; i < n; i++) {
      const d = x[i] - mean;
      varc += d * d;
    }
    varc /= n;
    const inv = 1 / Math.sqrt(varc + this.eps);
    for (let i = 0; i < n; i++) x[i] = (x[i] - mean) * inv * g[i] + b[i];
    return x;
  }

  // one block: Mish(LayerNorm(Linear(x)))
  _block(x, linW, linB, lnG, lnB) {
    const y = this._linear(x, linW, linB);
    this._layerNormInPlace(y, lnG, lnB);
    return mishInPlace(y);
  }

  _adapt(obs) {
    let h = this._block(obs, 'adapt.0.weight', 'adapt.0.bias', 'adapt.1.weight', 'adapt.1.bias');
    h = this._block(h, 'adapt.3.weight', 'adapt.3.bias', 'adapt.4.weight', 'adapt.4.bias');
    return this._linear(h, 'adapt.6.weight', 'adapt.6.bias'); // 256, no activation
  }

  _adaptJoint(obs) {
    let h = this._block(obs, 'adapt_joint.0.weight', 'adapt_joint.0.bias', 'adapt_joint.1.weight', 'adapt_joint.1.bias');
    h = this._block(h, 'adapt_joint.3.weight', 'adapt_joint.3.bias', 'adapt_joint.4.weight', 'adapt_joint.4.bias');
    return this._linear(h, 'adapt_joint.6.weight', 'adapt_joint.6.bias'); // 145
  }

  _actor(x658) {
    let h = this._block(x658, 'actor.fc0.weight', 'actor.fc0.bias', 'actor.ln0.weight', 'actor.ln0.bias');
    h = this._block(h, 'actor.fc1.weight', 'actor.fc1.bias', 'actor.ln1.weight', 'actor.ln1.bias');
    h = this._block(h, 'actor.fc2.weight', 'actor.fc2.bias', 'actor.ln2.weight', 'actor.ln2.bias');
    return this._linear(h, 'actor.mean.weight', 'actor.mean.bias'); // 29, no activation
  }

  /**
   * @param {Float32Array} obs  normalized observation (257)
   * @returns {Float32Array} raw action (29)
   */
  forward(obs) {
    const a = this._adapt(obs); // 256
    const aj = this._adaptJoint(obs); // 145
    const input = this._input658;
    input.set(obs, 0);
    input.set(a, 257);
    input.set(aj, 257 + 256);
    return this._actor(input);
  }
}
