"""TensorRT engine runner for the perception networks (Jetson GPU path).

The Jetson image ships TensorRT but no onnxruntime, so without this the nodes fall back to
OpenCV DNN on the CPU (near + far band ~39 ms per frame, over the 33 ms budget at 30 Hz).
With serialized engines (trtexec --fp16) both networks take ~1.3 ms together.

Engines are device-specific: build them on the robot, next to the ONNX file:
    trtexec --onnx=model.onnx --saveEngine=model.engine --fp16
`resolve_model()` then picks model.engine automatically when the ONNX path is given.

CUDA memory is handled through ctypes on libcudart, so the only Python dependency is the
`tensorrt` module itself (TensorRT 10+ tensor-address API: execute_async_v3).
"""
import ctypes
import ctypes.util
import glob
import os
import site
from pathlib import Path

import numpy as np

ENGINE_SUFFIXES = ('.engine', '.plan', '.trt')
H2D, D2H = 1, 2


def resolve_model(path, prefer_engine=True):
    """Return (path, is_engine). An ONNX path is swapped for a sibling engine if one exists."""
    p = Path(path)
    if p.suffix in ENGINE_SUFFIXES:
        return p, True
    if prefer_engine and os.environ.get('LF_NO_TRT') != '1':
        for suffix in ENGINE_SUFFIXES:
            e = p.with_suffix(suffix)
            if e.is_file():
                return e, True
    return p, False


def _load_cudart():
    names = ['libcudart.so', 'libcudart.so.13', 'libcudart.so.12', 'libcudart.so.11.0']
    found = ctypes.util.find_library('cudart')
    if found:
        names.insert(0, found)
    dirs = ['/usr/local/cuda/lib64', '/usr/local/cuda/targets/aarch64-linux/lib']
    for sp in site.getsitepackages() + [site.getusersitepackages()]:
        dirs += glob.glob(os.path.join(sp, 'nvidia', 'cuda_runtime', 'lib'))
        dirs += glob.glob(os.path.join(sp, 'nvidia', 'cu*', 'lib'))
    errors = []
    for name in names + [os.path.join(d, n) for d in dirs for n in names]:
        try:
            lib = ctypes.CDLL(name)
        except OSError as e:
            errors.append(str(e))
            continue
        lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        lib.cudaFree.argtypes = [ctypes.c_void_p]
        lib.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                        ctypes.c_int, ctypes.c_void_p]
        lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        return lib
    raise OSError('libcudart not found: ' + '; '.join(errors[-3:]))


def _check(code, what):
    if code != 0:
        raise RuntimeError(f'CUDA error {code} in {what}')


class TrtRunner:
    """run(feeds) -> {output name: array}; outputs also available in engine order (output_names)."""

    def __init__(self, engine_path):
        import tensorrt as trt
        self.trt = trt
        self.cuda = _load_cudart()
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(engine_path).read_bytes())
        if self.engine is None:
            raise RuntimeError(f'cannot deserialize TensorRT engine {engine_path} '
                               '(built for another GPU or TensorRT version?)')
        self.ctx = self.engine.create_execution_context()
        stream = ctypes.c_void_p()
        _check(self.cuda.cudaStreamCreate(ctypes.byref(stream)), 'cudaStreamCreate')
        self.stream = stream
        self.inputs, self.output_names, self.host, self.dev = [], [], {}, {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            if any(d < 0 for d in shape):
                raise ValueError(f'dynamic shape {shape} for {name}: build the engine with fixed shapes')
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            self.host[name] = np.empty(shape, dtype)
            ptr = ctypes.c_void_p()
            _check(self.cuda.cudaMalloc(ctypes.byref(ptr), self.host[name].nbytes), 'cudaMalloc')
            self.dev[name] = ptr
            self.ctx.set_tensor_address(name, ptr.value)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append(name)
            else:
                self.output_names.append(name)

    def input_shape(self, name=None):
        return self.host[name or self.inputs[0]].shape

    def run(self, feeds):
        if not isinstance(feeds, dict):
            feeds = {self.inputs[0]: feeds}
        for name in self.inputs:
            buf = self.host[name]
            buf[...] = np.asarray(feeds[name]).reshape(buf.shape)   # dtype cast (fp16 IO engines)
            _check(self.cuda.cudaMemcpyAsync(self.dev[name], buf.ctypes.data, buf.nbytes, H2D,
                                             self.stream), 'H2D')
        if not self.ctx.execute_async_v3(self.stream.value):
            raise RuntimeError('TensorRT execute_async_v3 failed')
        for name in self.output_names:
            buf = self.host[name]
            _check(self.cuda.cudaMemcpyAsync(buf.ctypes.data, self.dev[name], buf.nbytes, D2H,
                                             self.stream), 'D2H')
        _check(self.cuda.cudaStreamSynchronize(self.stream), 'cudaStreamSynchronize')
        return {n: self.host[n].astype(np.float32, copy=True) for n in self.output_names}

    def __del__(self):
        cuda = getattr(self, 'cuda', None)
        if cuda is None:
            return
        for ptr in getattr(self, 'dev', {}).values():
            cuda.cudaFree(ptr)
        if getattr(self, 'stream', None) is not None:
            cuda.cudaStreamDestroy(self.stream)
