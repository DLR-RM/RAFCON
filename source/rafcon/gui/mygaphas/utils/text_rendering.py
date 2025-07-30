"""Text rendering safety utilities for extreme zoom levels"""

MAX_SAFE_FONT_SIZE = 16000
MAX_ZOOM_LEVEL = 500000

MIN_ZOOM_SCALE = 0.0001
MAX_ZOOM_SCALE = 1000.0

MIN_VISIBLE_FONT_SIZE = 0.5


def clamp_zoom_scale(zoom_scale):
    """Clamp zoom scale to bounds"""
    return max(MIN_ZOOM_SCALE, min(zoom_scale, MAX_ZOOM_SCALE))


def should_render_text(font_size, zoom_scale, current_zoom):
    """Check if text should be rendered based on font size and zoom"""
    if current_zoom > MAX_ZOOM_LEVEL:
        return False
    
    if zoom_scale > 0:
        effective_size = font_size / zoom_scale
    else:
        return False
    
    # Check visual size
    visual_size = font_size * current_zoom
    if visual_size < MIN_VISIBLE_FONT_SIZE:
        return False
        
    # Check effective size against limit
    return effective_size <= MAX_SAFE_FONT_SIZE * 0.95