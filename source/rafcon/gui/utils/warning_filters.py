#!/usr/bin/env python3

import os
import re
import threading
import atexit
import platform
import fcntl
import logging

logger = logging.getLogger(__name__)


def setup_pango_warning_filter():
    """Sets up filtering for Pango warnings on Ubuntu systems.
    
    This function filters out specific Pango warnings that are noisy but harmless,
    particularly warnings about failed cairo scaled fonts that occur during normal operation.
    The filter only runs on Ubuntu systems to avoid interfering with other platforms.
    """
    # Only run on Ubuntu
    if platform.system() != 'Linux':
        return
    
    try:
        # Check if this is Ubuntu by reading /etc/os-release
        if os.path.exists('/etc/os-release'):
            with open('/etc/os-release', 'r') as f:
                content = f.read()
                if 'Ubuntu' not in content:
                    return
        else:
            return
    except Exception:
        return
    
    try:
        original_stderr = os.dup(2)
        r, w = os.pipe()
        os.dup2(w, 2)
        os.close(w)
        
        # Make read end non-blocking
        fcntl.fcntl(r, fcntl.F_SETFL, fcntl.fcntl(r, fcntl.F_GETFL) | os.O_NONBLOCK)
        
        filter_running = threading.Event()
        filter_running.set()
        
        def filter_pango_warnings():
            pango_pattern = re.compile(rb'Pango-WARNING.*(?:failed to create cairo scaled font|font_face status|scaled_font status)')
            buffer = b''
            suppress_next_empty = False
            
            while filter_running.is_set():
                try:
                    data = os.read(r, 4096)
                    if not data:
                        break
                    buffer += data
                    
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        
                        if pango_pattern.search(line):
                            suppress_next_empty = True
                            continue
                        
                        if suppress_next_empty and not line.strip():
                            suppress_next_empty = False
                            continue
                        
                        suppress_next_empty = False
                        os.write(original_stderr, line + b'\n')
                        
                except BlockingIOError:
                    threading.Event().wait(0.01)
                except Exception:
                    break
            
            if buffer and not pango_pattern.search(buffer):
                os.write(original_stderr, buffer)
            os.close(r)
        
        threading.Thread(target=filter_pango_warnings, daemon=True).start()
        
        def restore_stderr():
            filter_running.clear()
            os.dup2(original_stderr, 2)
            os.close(original_stderr)
        
        atexit.register(restore_stderr)
        
    except Exception as e:
        logger.debug("Failed to setup Pango warning filter: %s", e)
