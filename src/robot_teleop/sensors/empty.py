class EmptyCameraGroup:
    """Explicit zero-camera source group; imports no camera or image libraries."""
    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        pass

    def close(self):
        pass

    def latest_frames(self):
        return {}

    def diagnostics(self):
        return {"errors": [], "cameras": {}, "running_workers": []}
