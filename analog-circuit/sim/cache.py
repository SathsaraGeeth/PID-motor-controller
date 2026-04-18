import hashlib
import pickle
import os
import functools


class Cache:
    def __init__(self, cache_dir: str = ".pid_cache"):
        self.cache_dir = cache_dir
        os.makedirs(cache_dir, exist_ok=True)
    def __call__(self, func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            key  = self._make_key(func, args, kwargs)
            path = self._path(key)
            if os.path.exists(path):
                with open(path, "rb") as f:
                    print(f"   [cache] hit  — {func.__name__} ({key[:8]}...)")
                    return pickle.load(f)
            print(f"   [cache] miss — {func.__name__} ({key[:8]}...)")
            result = func(*args, **kwargs)
            with open(path, "wb") as f:
                pickle.dump(result, f)
            return result
        return wrapper
    def invalidate(self, func=None):
        removed = 0
        for fname in os.listdir(self.cache_dir):
            if func is None or fname.startswith(func.__name__ + "_"):
                os.remove(os.path.join(self.cache_dir, fname))
                removed += 1
        print(f"   [cache] invalidated {removed} entr{'y' if removed == 1 else 'ies'}")
    def _make_key(self, func, args, kwargs) -> str:
        payload = pickle.dumps((func.__name__, args, kwargs))
        return func.__name__ + "_" + hashlib.sha256(payload).hexdigest()
    def _path(self, key: str) -> str:
        return os.path.join(self.cache_dir, key + ".pkl")
        