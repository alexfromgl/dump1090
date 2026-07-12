function time() {
    var d = new Date();
    var s = d.getSeconds();
    var m = d.getMinutes();
    var h = d.getHours();
    return ("0" + h).substr(-2) + ":" + ("0" + m).substr(-2) + ":" + ("0" + s).substr(-2);
}

function getCardinalDirection(angle) {
    if (typeof angle === 'string') angle = parseInt(angle);
    if (angle < 0 || angle > 360 || typeof angle === 'undefined') return '☈';
    const arrows = { north: '↑ N', north_east: '↗ NE', east: '→ E', south_east: '↘ SE', south: '↓ S', south_west: '↙ SW', west: '← W', north_west: '↖ NW' };
    const directions = Object.keys(arrows);
    const degree = 360 / directions.length;
    angle = angle + degree / 2;
    for (let i = 0; i < directions.length; i++) {
        if (angle >= (i * degree) && angle < (i + 1) * degree) return arrows[directions[i]];
    }
    return arrows['north'];
}

// Leaflet map instance and planes state
let map = null;
const planes = {};
let selectedPlane = null;
let copyingHex = null; // tracks which plane's info is being shown as "copied"


const escapeHtml = (value) => String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#039;');

const updateFlightsTable = () => {
    const tableBody = document.getElementById('flights-table-body');
    const count = document.getElementById('flights-count');
    const activeFlights = Object.values(planes).sort((a, b) => {
        const aName = a.flight || a.hex;
        const bName = b.flight || b.hex;
        return aName.localeCompare(bName);
    });

    count.textContent = activeFlights.length;

    if (activeFlights.length === 0) {
        tableBody.innerHTML = '<tr><td colspan="5" class="flights-table__empty">No active aircraft</td></tr>';
        return;
    }

    tableBody.innerHTML = activeFlights.map((plane) => {
        const flight = escapeHtml(plane.flight || 'Unknown');
        const hex = escapeHtml(plane.hex);
        const altitude = plane.altitude != null ? `${plane.altitude} m` : '—';
        const speed = plane.speed != null ? `${plane.speed} km/h` : '—';
        const heading = plane.track != null
            ? `${plane.track}° ${getCardinalDirection(plane.track)}`
            : '—';
        const latitude = Number.isFinite(Number(plane.lat))
            ? Number(plane.lat).toFixed(4)
            : '—';
        const longitude = Number.isFinite(Number(plane.lon))
            ? Number(plane.lon).toFixed(4)
            : '—';
        const selectedClass = selectedPlane?.hex === plane.hex
            ? ' class="flights-table__row--selected"'
            : '';

        return `
            <tr data-hex="${hex}"${selectedClass} title="Select ${flight}">
                <td>
                    <span class="flights-table__flight">${flight}</span>
                    <span class="flights-table__icao">${hex}</span>
                </td>
                <td>${escapeHtml(altitude)}</td>
                <td>${escapeHtml(speed)}</td>
                <td>${escapeHtml(heading)}</td>
                <td class="flights-table__coords">
                    <span>${escapeHtml(latitude)}</span>
                    <span>${escapeHtml(longitude)}</span>
                </td>
            </tr>
        `;
    }).join('');
};

// Update panel with new plane info
const updatePlaneInfo = (newPlane) => {
    if (!newPlane || copyingHex === newPlane.hex) return;

    const flight = escapeHtml(newPlane.flight || '—');
    const hex = escapeHtml(newPlane.hex);
    const heading = newPlane.track != null
        ? `${newPlane.track}° ${getCardinalDirection(newPlane.track)}`
        : '—';
    const latitude = Number(newPlane.lat).toFixed(4);
    const longitude = Number(newPlane.lon).toFixed(4);

    const el = document.getElementById('planeInfo');
    el.innerHTML = `
        <div class="info-panel__flight" data-copy="${flight}" onclick="copyField(this)" title="Click to copy">${flight}</div>
        <div class="info-panel__icao" data-copy="${hex}" title="Click to copy">${hex}</div>
        <div class="info-panel__grid">
            <div class="info-panel__item">
                <span class="info-panel__label">Altitude</span>
                <span class="info-panel__value" data-copy="${newPlane.altitude ?? '—'}">${newPlane.altitude ?? '—'} m</span>
            </div>
            <div class="info-panel__item">
                <span class="info-panel__label">Speed</span>
                <span class="info-panel__value" data-copy="${newPlane.speed ?? '—'}">${newPlane.speed ?? '—'} km/h</span>
            </div>
            <div class="info-panel__item">
                <span class="info-panel__label">Heading</span>
                <span class="info-panel__value" data-copy="${newPlane.track ?? '—'}">${escapeHtml(heading)}</span>
            </div>
        </div>
        <div class="info-panel__coords" data-copy="${latitude}, ${longitude}" onclick="copyField(this)" title="Click to copy">${latitude}, ${longitude}</div>
    `;
};

// Copy text to clipboard when clicking on value elements
// Defines copyField function on the global window object to be accessible from inline onclick handlers
window.copyField = (el) => {
    const text = el.dataset.copy;
    if (!text || text === '—') return;

    copyingHex = selectedPlane?.hex;

    const finish = () => {
        const original = el.innerHTML;
        el.innerHTML = 'Copied!';
        setTimeout(() => {
            el.innerHTML = original;
            copyingHex = null;
        }, 1500);
    };

    if (navigator.clipboard) {
        navigator.clipboard.writeText(text).then(finish);
    } else {
        // non-secure contexts (not on https or localhost)
        const ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed';
        ta.style.opacity = '0';
        document.body.appendChild(ta);
        ta.select();
        document.execCommand('copy');
        document.body.removeChild(ta);
        finish();
    }
};

// Create a custom marker with Font Awesome plane icon
const getIconForPlane = (plane) => {
    const icon = L.divIcon({
        html: `
            <div
                class="plane-icon__symbol${selectedPlane?.hex === plane.hex ? ' plane-icon__symbol--selected' : ''}"
                style="transform: rotate(${plane.track ?? 0}deg);"
            >
                <i class="fa-solid fa-plane-up"></i>
            </div>
            ${plane.flight ? `<div class="plane-icon__flight">${escapeHtml(plane.flight)}</div>` : ''}
            `,
        className: 'plane-icon',
        iconSize: [32, 32],
        iconAnchor: [16, 16]
    });
    return icon;
};

// Select a new plane
const selectPlane = (newPlane) => {
    // Exit if new plane is not found
    if (!newPlane) return;

    // Update plane selection
    const oldPlane = planes[selectedPlane?.hex];
    selectedPlane = newPlane;

    // Remove highlighting from the previously selected plane
    oldPlane?.marker.setIcon(getIconForPlane(oldPlane));

    // Update currently selected plane
    newPlane.marker.setIcon(getIconForPlane(newPlane));
    updatePlaneInfo(newPlane);
    updateFlightsTable();
};

// Fetch planes data from the server
const fetchData = async () => {
    try {
        const response = await fetch('/data');
        if (!response.ok) return;
        const data = await response.json();
        const activePlanes = {};
        for (const plane of data) {
            // Skip planes without valid coordinates
            if (plane.lat == null || plane.lon == null) continue;

            // Build the plane list from the fetched data
            let marker = null;
            activePlanes[plane.hex] = true;
            plane.flight = plane.flight?.trim() || '';
            const existingPlane = planes[plane.hex];
            if (existingPlane) {
                // Update existing plane marker position
                marker = existingPlane.marker;
                marker.setLatLng([plane.lat, plane.lon]);

                // The marker HTML must be rebuilt whenever a value shown
                // inside the icon changes. Previously it was rebuilt only when
                // the track changed, so a newly received flight number remained
                // hidden until clicking the marker.
                const flightChanged = existingPlane.flight !== plane.flight;
                let trackChanged = existingPlane.track !== plane.track;

                if (existingPlane.track != null && plane.track != null) {
                    let trackDelta = Math.abs(existingPlane.track - plane.track);
                    if (trackDelta > 180) trackDelta = 360 - trackDelta;
                    trackChanged = trackDelta > 5;
                }

                // Copy every current aircraft value while preserving
                // the existing Leaflet marker object.
                Object.assign(existingPlane, plane);
                existingPlane.marker = marker;

                if (flightChanged || trackChanged) {
                    marker.setIcon(getIconForPlane(existingPlane));
                }

                // Update existing plane info in the panel if it is selected
                if (existingPlane.hex === selectedPlane?.hex) {
                    updatePlaneInfo(existingPlane);
                }
            } else {
                // Create a new marker for new planes
                const icon = getIconForPlane(plane);
                marker = L.marker([plane.lat, plane.lon], { icon: icon }).addTo(map);
                plane.marker = marker;
                planes[plane.hex] = plane;
                marker.on('click', () => selectPlane(planes[plane.hex]));
            }
        }

        // Remove idle planes
        for (const plane of Object.values(planes)) {
            if (!activePlanes[plane.hex]) {
                map.removeLayer(plane.marker);
                delete planes[plane.hex];
                if (selectedPlane?.hex === plane.hex) {
                    selectedPlane = null;
                    document.getElementById('planeInfo').innerHTML = 'Click on a plane for info';
                }
            }
        }

        updateFlightsTable();

        let lastUpdate = document.getElementById('lastUpdate');
        lastUpdate.innerText = time();
    } catch (error) {
        console.error('Error fetching data:', error);
    }
};

document.getElementById('flights-table-body').addEventListener('click', (event) => {
    const row = event.target.closest('tr[data-hex]');
    if (!row) return;

    const plane = planes[row.dataset.hex];
    if (!plane) return;

    selectPlane(plane);
    map.panTo([plane.lat, plane.lon]);
});

// Make active flights panel draggable
interact('#flights-panel').draggable({
    allowFrom: '.flights-panel__header',
    listeners: {
        move: (event) => {
            const target = event.target;
            const x = (parseFloat(target.dataset.x) || 0) + event.dx;
            const y = (parseFloat(target.dataset.y) || 0) + event.dy;
            target.style.transform = `translate(${x}px, ${y}px)`;
            target.dataset.x = x;
            target.dataset.y = y;
        }
    }
});

// Make info panel draggable
interact('#info-panel').draggable({
    allowFrom: '.info-panel__header',
    listeners: {
        move: (event) => {
            const target = event.target;
            const x = (parseFloat(target.dataset.x) || 0) + event.dx;
            const y = (parseFloat(target.dataset.y) || 0) + event.dy;
            target.style.transform = `translate(${x}px, ${y}px)`;
            target.dataset.x = x;
            target.dataset.y = y;
        }
    }
});

// Initialize Leaflet instance
const savedView = JSON.parse(localStorage.getItem('mapView'));
map = L.map('canvas', { zoomControl: false }).setView(
    savedView ? [savedView.lat, savedView.lng] : [53.644230, -2.654571],
    savedView ? savedView.zoom : 10
);

// Keep the top-left free for the active flights panel.
L.control.zoom({ position: 'bottomleft' }).addTo(map);

L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

map.on('moveend', () => {
    const center = map.getCenter();
    localStorage.setItem('mapView', JSON.stringify({
        lat: center.lat,
        lng: center.lng,
        zoom: map.getZoom()
    }));
});

// Load immediately, then continue polling the server.
fetchData();
setInterval(fetchData, 200);