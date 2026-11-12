"""
Map tile servers used by the desktop map (mapping.TileManager). Qt-free so the web interface can list them.
"""

# Default tile server configuration
TILE_SERVERS = {
    'OpenStreetMap': {
        'url': 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
        'backup_urls': [
            'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png',
            'https://b.tile.openstreetmap.org/{z}/{x}/{y}.png',
            'https://c.tile.openstreetmap.org/{z}/{x}/{y}.png'
        ],
        'user_agent': 'GPSTracker/1.0'
    },
    'OpenTopoMap': {
        'url': 'https://tile.opentopomap.org/{z}/{x}/{y}.png',
        'user_agent': 'GPSTracker/1.0'
    },
    'CyclOSM': {
        'url': 'https://c.tile-cyclosm.openstreetmap.fr/cyclosm/{z}/{x}/{y}.png',
        'user_agent': 'GPSTracker/1.0'
    }
}
