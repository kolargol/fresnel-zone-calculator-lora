// ============================================================
// Fresnel Zone Calculator - 868 MHz LoRa Link Analysis
// ============================================================

(function () {
    'use strict';

    // --- Constants ---
    const CONFIG = {
        frequency: 868e6,           // 868 MHz
        speedOfLight: 299792458,    // m/s
        earthRadius: 6371000,       // meters
        kFactor: 4 / 3,            // Standard atmosphere refraction
        defaultCenter: [50.0647, 19.9450], // Krakow
        defaultZoom: 13,
        pathSamples: 200,          // Number of sample points along path
        buildingBuffer: 100,       // meters buffer for building query
        defaultBuildingHeight: 10, // meters if no height data
        floorHeight: 3,           // meters per floor
    };

    CONFIG.wavelength = CONFIG.speedOfLight / CONFIG.frequency; // ~0.3456m

    // --- State ---
    const state = {
        map: null,
        txMarker: null,
        rxMarker: null,
        txPoint: null,
        rxPoint: null,
        pathLine: null,
        fresnelPolygon: null,
        placingMode: 'tx', // 'tx', 'rx', 'done'
        profileData: null,
        buildingData: [],
    };

    // --- Utility Functions ---

    function toRad(deg) { return deg * Math.PI / 180; }
    function toDeg(rad) { return rad * 180 / Math.PI; }

    function haversineDistance(lat1, lon1, lat2, lon2) {
        const dLat = toRad(lat2 - lat1);
        const dLon = toRad(lon2 - lon1);
        const a = Math.sin(dLat / 2) ** 2 +
            Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
        return 2 * CONFIG.earthRadius * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    function interpolatePoints(p1, p2, n) {
        const points = [];
        for (let i = 0; i <= n; i++) {
            const t = i / n;
            points.push({
                lat: p1.lat + (p2.lat - p1.lat) * t,
                lng: p1.lng + (p2.lng - p1.lng) * t,
            });
        }
        return points;
    }

    function fresnelRadius(n, d1, d2, wavelength) {
        const D = d1 + d2;
        if (D === 0) return 0;
        return Math.sqrt(n * wavelength * d1 * d2 / D);
    }

    function freeSpacePathLoss(distance, frequency) {
        if (distance <= 0) return 0;
        return 20 * Math.log10(distance) + 20 * Math.log10(frequency) + 20 * Math.log10(4 * Math.PI / CONFIG.speedOfLight);
    }

    function earthCurvatureHeight(d1, d2) {
        return (d1 * d2) / (2 * CONFIG.kFactor * CONFIG.earthRadius);
    }

    function pointInPolygon(point, polygon) {
        let inside = false;
        const x = point.lng, y = point.lat;
        for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            const xi = polygon[i].lon, yi = polygon[i].lat;
            const xj = polygon[j].lon, yj = polygon[j].lat;
            const intersect = ((yi > y) !== (yj > y)) &&
                (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    // Line-segment intersection test (2D)
    function lineSegmentIntersects(p1, p2, p3, p4) {
        // Returns true if segment p1-p2 intersects segment p3-p4
        const d1x = p2.lng - p1.lng, d1y = p2.lat - p1.lat;
        const d2x = p4.lng - p3.lng, d2y = p4.lat - p3.lat;
        const denom = d1x * d2y - d1y * d2x;
        if (Math.abs(denom) < 1e-12) return false;
        const t = ((p3.lng - p1.lng) * d2y - (p3.lat - p1.lat) * d2x) / denom;
        const u = ((p3.lng - p1.lng) * d1y - (p3.lat - p1.lat) * d1x) / denom;
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    // Check if the path line (from txPoint to rxPoint) crosses a building polygon
    function pathCrossesBuilding(txPt, rxPt, polygon) {
        const p1 = { lat: txPt.lat, lng: txPt.lng };
        const p2 = { lat: rxPt.lat, lng: rxPt.lng };
        for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            const p3 = { lat: polygon[j].lat, lng: polygon[j].lon };
            const p4 = { lat: polygon[i].lat, lng: polygon[i].lon };
            if (lineSegmentIntersects(p1, p2, p3, p4)) return true;
        }
        return false;
    }

    // Project a point onto the line from A to B, return fraction [0..1]
    function projectOntoLine(point, A, B) {
        const dx = B.lng - A.lng, dy = B.lat - A.lat;
        const len2 = dx * dx + dy * dy;
        if (len2 === 0) return 0;
        const t = ((point.lng - A.lng) * dx + (point.lat - A.lat) * dy) / len2;
        return Math.max(0, Math.min(1, t));
    }

    // --- API Services ---

    async function fetchElevation(points) {
        const batchSize = 100;
        const allElevations = [];

        for (let i = 0; i < points.length; i += batchSize) {
            const batch = points.slice(i, i + batchSize);
            const lats = batch.map(p => p.lat.toFixed(6)).join(',');
            const lngs = batch.map(p => p.lng.toFixed(6)).join(',');
            const url = `https://api.open-meteo.com/v1/elevation?latitude=${lats}&longitude=${lngs}`;

            const response = await fetch(url);
            if (!response.ok) throw new Error(`Elevation API error: ${response.status}`);
            const data = await response.json();
            allElevations.push(...data.elevation);
        }

        return allElevations;
    }

    async function fetchBuildings(points, totalDistance) {
        // Create a polyline from sampled points for Overpass query
        const sampleStep = Math.max(1, Math.floor(points.length / 20));
        const queryPoints = [];
        for (let i = 0; i < points.length; i += sampleStep) {
            queryPoints.push(points[i]);
        }
        queryPoints.push(points[points.length - 1]);

        const buffer = CONFIG.buildingBuffer;

        const query = `[out:json][timeout:30];
(
  way["building"](around:${buffer},${queryPoints.map(p => `${p.lat},${p.lng}`).join(',')});
  relation["building"](around:${buffer},${queryPoints.map(p => `${p.lat},${p.lng}`).join(',')});
);
out body geom;`;

        try {
            const response = await fetch('https://overpass-api.de/api/interpreter', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'data=' + encodeURIComponent(query),
            });

            if (!response.ok) throw new Error(`Overpass API error: ${response.status}`);
            const data = await response.json();
            const elements = data.elements || [];
            // Normalize: relations have members with geometry, extract outer way geometry
            const normalized = [];
            for (const el of elements) {
                if (el.type === 'way' && el.geometry) {
                    normalized.push(el);
                } else if (el.type === 'relation' && el.members) {
                    // Use the first outer member's geometry as the building footprint
                    for (const m of el.members) {
                        if (m.role === 'outer' && m.geometry) {
                            normalized.push({ ...el, geometry: m.geometry });
                            break;
                        }
                    }
                }
            }
            return normalized;
        } catch (e) {
            console.warn('Building data unavailable:', e.message);
            return [];
        }
    }

    function getBuildingHeight(element) {
        const tags = element.tags || {};
        if (tags.height) {
            const h = parseFloat(tags.height);
            if (!isNaN(h)) return h;
        }
        if (tags['building:height']) {
            const h = parseFloat(tags['building:height']);
            if (!isNaN(h)) return h;
        }
        if (tags['building:levels']) {
            const levels = parseInt(tags['building:levels']);
            if (!isNaN(levels)) return levels * CONFIG.floorHeight;
        }
        // Default heights by building type
        const type = tags.building;
        if (type === 'church' || type === 'cathedral') return 25;
        if (type === 'industrial' || type === 'warehouse') return 12;
        if (type === 'apartments' || type === 'residential') return 15;
        if (type === 'commercial' || type === 'office') return 20;
        return CONFIG.defaultBuildingHeight;
    }

    // --- Fresnel Zone Calculation ---

    function calculateProfile(points, elevations, buildings, totalDistance, txPt, rxPt) {
        const numPoints = points.length;
        const profile = [];

        // Pre-process buildings into polygons with heights
        const buildingPolygons = buildings.map(b => ({
            geometry: b.geometry,
            height: getBuildingHeight(b),
            tags: b.tags || {},
        }));

        // Pre-compute which buildings are actually crossed by the path line
        // This is more reliable than point-in-polygon sampling alone
        const crossedBuildings = [];
        for (const bld of buildingPolygons) {
            if (bld.geometry && pathCrossesBuilding(txPt, rxPt, bld.geometry)) {
                // Compute centroid for distance-along-path projection
                let cLat = 0, cLng = 0;
                for (const v of bld.geometry) { cLat += v.lat; cLng += v.lon; }
                cLat /= bld.geometry.length;
                cLng /= bld.geometry.length;
                const t = projectOntoLine({ lat: cLat, lng: cLng }, txPt, rxPt);
                crossedBuildings.push({ ...bld, t, centroidLat: cLat, centroidLng: cLng });
            }
        }

        for (let i = 0; i < numPoints; i++) {
            const d = (i / (numPoints - 1)) * totalDistance;
            const d1 = d;
            const d2 = totalDistance - d;
            const terrain = elevations[i] || 0;
            const curvature = earthCurvatureHeight(d1, d2);
            const t = i / (numPoints - 1);

            // Check if this point is inside any building (point-in-polygon)
            let buildingHeight = 0;
            let inBuilding = false;
            for (const bld of buildingPolygons) {
                if (bld.geometry && pointInPolygon(points[i], bld.geometry)) {
                    buildingHeight = Math.max(buildingHeight, bld.height);
                    inBuilding = true;
                }
            }

            // Also check if any crossed building covers this fraction of the path
            // (catches buildings missed by discrete point-in-polygon sampling)
            if (!inBuilding) {
                for (const cb of crossedBuildings) {
                    // Estimate building width along path as ~30m / totalDistance fraction
                    const bldWidthFraction = 30 / totalDistance;
                    if (Math.abs(t - cb.t) < bldWidthFraction) {
                        // Verify by checking proximity to building centroid
                        const dist = haversineDistance(points[i].lat, points[i].lng, cb.centroidLat, cb.centroidLng);
                        if (dist < 40) {
                            buildingHeight = Math.max(buildingHeight, cb.height);
                            inBuilding = true;
                        }
                    }
                }
            }

            const effectiveHeight = terrain + curvature + buildingHeight;
            const f1 = fresnelRadius(1, d1, d2, CONFIG.wavelength);

            profile.push({
                distance: d,
                terrain: terrain,
                curvature: curvature,
                buildingHeight: buildingHeight,
                inBuilding: inBuilding,
                effectiveHeight: effectiveHeight,
                fresnelRadius: f1,
                lat: points[i].lat,
                lng: points[i].lng,
            });
        }

        return profile;
    }

    function analyzeLink(profile, txHeight, rxHeight, txPower, txGain, rxGain, rxSensitivity) {
        const totalDistance = profile[profile.length - 1].distance;
        const txElev = profile[0].terrain + profile[0].curvature;
        const rxElev = profile[profile.length - 1].terrain + profile[profile.length - 1].curvature;
        const txAltitude = txElev + txHeight;
        const rxAltitude = rxElev + rxHeight;

        let worstClearance = Infinity;
        let worstClearancePct = Infinity;

        for (let i = 1; i < profile.length - 1; i++) {
            const p = profile[i];
            const t = p.distance / totalDistance;
            const losHeight = txAltitude + (rxAltitude - txAltitude) * t;
            const clearance = losHeight - p.effectiveHeight;
            const clearancePct = p.fresnelRadius > 0 ? (clearance / p.fresnelRadius) * 100 : 100;

            if (clearancePct < worstClearancePct) {
                worstClearancePct = clearancePct;
                worstClearance = clearance;
            }
        }

        // Calculate obstruction loss — accumulate from each distinct obstruction group
        let obstructionLoss = 0;
        let numObstructions = 0;
        let inObstruction = false;
        let currentObs = { maxPenetration: 0, d1: 0, d2: 0, isBuilding: false };

        for (let i = 1; i < profile.length - 1; i++) {
            const p = profile[i];
            const t = p.distance / totalDistance;
            const losHeight = txAltitude + (rxAltitude - txAltitude) * t;
            const clearance = losHeight - p.effectiveHeight;

            if (clearance < 0) {
                // This point is obstructed (obstacle above LOS line)
                const penetration = Math.abs(clearance);
                if (!inObstruction) {
                    inObstruction = true;
                    currentObs = { maxPenetration: penetration, d1: p.distance, d2: totalDistance - p.distance, isBuilding: p.inBuilding };
                } else {
                    if (penetration > currentObs.maxPenetration) {
                        currentObs.maxPenetration = penetration;
                        currentObs.d1 = p.distance;
                        currentObs.d2 = totalDistance - p.distance;
                    }
                    if (p.inBuilding) currentObs.isBuilding = true;
                }
            } else {
                if (inObstruction) {
                    // End of this obstruction — calculate knife-edge diffraction loss
                    obstructionLoss += calcObstructionLoss(currentObs);
                    numObstructions++;
                    inObstruction = false;
                }
            }
        }
        // Handle if path ends in obstruction
        if (inObstruction) {
            obstructionLoss += calcObstructionLoss(currentObs);
            numObstructions++;
        }

        // For partial Fresnel zone obstruction (terrain within zone but below LOS)
        if (obstructionLoss === 0 && worstClearancePct < 100) {
            obstructionLoss = Math.max(0, (1 - worstClearancePct / 100) * 6);
        }

        const fspl = freeSpacePathLoss(totalDistance, CONFIG.frequency);
        const totalLoss = fspl + obstructionLoss;
        const receivedPower = txPower + txGain + rxGain - totalLoss;
        const linkMargin = receivedPower - rxSensitivity;

        // Fresnel zone max radius (at midpoint)
        const maxFresnelR = fresnelRadius(1, totalDistance / 2, totalDistance / 2, CONFIG.wavelength);

        // Number of obstructions for display
        const obstructionCount = numObstructions;

        // Signal quality assessment
        let quality, qualityClass, description, recommendations;
        if (linkMargin >= 20) {
            quality = 'Excellent';
            qualityClass = 'signal-excellent';
            description = 'Strong and reliable link with high margin';
            recommendations = 'Link is well within operational parameters. Consider reducing TX power to save energy.';
        } else if (linkMargin >= 10) {
            quality = 'Good';
            qualityClass = 'signal-good';
            description = 'Reliable link under normal conditions';
            recommendations = 'Link is stable. Minor weather effects should not disrupt communication.';
        } else if (linkMargin >= 3) {
            quality = 'Fair';
            qualityClass = 'signal-fair';
            description = 'Link may experience intermittent issues';
            recommendations = 'Consider increasing antenna height, using higher gain antennas, or reducing distance. Weather may cause dropouts.';
        } else if (linkMargin >= 0) {
            quality = 'Poor';
            qualityClass = 'signal-poor';
            description = 'Link at receiver sensitivity limit';
            recommendations = 'Unreliable link. Increase antenna heights, use directional antennas, or select higher spreading factor for better sensitivity.';
        } else {
            quality = 'No Link';
            qualityClass = 'signal-none';
            description = 'Signal below receiver sensitivity';
            recommendations = 'Link not viable with current parameters. Significantly increase antenna height, use high-gain antennas, increase TX power, or reduce distance.';
        }

        return {
            totalDistance,
            fspl,
            obstructionLoss,
            totalLoss,
            receivedPower,
            linkMargin,
            maxFresnelR,
            worstClearance,
            worstClearancePct,
            obstructionCount,
            quality,
            qualityClass,
            description,
            recommendations,
            txAltitude,
            rxAltitude,
        };
    }

    // Calculate diffraction/obstruction loss for a single obstacle group
    function calcObstructionLoss(obs) {
        const { maxPenetration, d1, d2, isBuilding } = obs;
        // Knife-edge diffraction parameter: v = h * sqrt(2*(d1+d2) / (λ*d1*d2))
        const D = d1 + d2;
        // Avoid division by zero near endpoints
        const effD1 = Math.max(d1, 1);
        const effD2 = Math.max(d2, 1);
        const v = maxPenetration * Math.sqrt(2 * D / (CONFIG.wavelength * effD1 * effD2));
        // ITU-R P.526 knife-edge approximation
        let loss = 6.9 + 20 * Math.log10(Math.sqrt((v - 0.1) ** 2 + 1) + v - 0.1);
        loss = Math.max(0, loss);
        // Buildings are NOT thin knife edges — add penetration loss
        // Typical building adds 15-25 dB extra on top of diffraction
        if (isBuilding) {
            loss = Math.max(loss, 18); // minimum 18 dB per building
            // Extra depth penalty (buildings are ~10-20m thick)
            loss += 5;
        }
        return loss;
    }

    // --- Profile Renderer ---

    function renderProfile(canvas, profile, analysis) {
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;

        const ctx = canvas.getContext('2d');
        ctx.scale(dpr, dpr);

        const w = rect.width;
        const h = rect.height;
        const pad = { top: 16, right: 20, bottom: 32, left: 58 };
        const drawW = w - pad.left - pad.right;
        const drawH = h - pad.top - pad.bottom;

        const totalDist = profile[profile.length - 1].distance;

        // Calculate Y range
        let minElev = Infinity, maxElev = -Infinity;
        for (const p of profile) {
            const terrainH = p.terrain + p.curvature;
            minElev = Math.min(minElev, terrainH);
            maxElev = Math.max(maxElev, terrainH + p.buildingHeight);
        }
        // Include LOS and Fresnel zone in range
        const losMax = Math.max(analysis.txAltitude, analysis.rxAltitude);
        const maxFR = analysis.maxFresnelR;
        maxElev = Math.max(maxElev, losMax + maxFR + 5);
        minElev = Math.max(0, minElev - 10);
        const elevRange = maxElev - minElev || 1;

        function xPos(d) { return pad.left + (d / totalDist) * drawW; }
        function yPos(elev) { return pad.top + drawH - ((elev - minElev) / elevRange) * drawH; }

        // Background
        ctx.fillStyle = '#0d1117';
        ctx.fillRect(0, 0, w, h);

        // Grid
        ctx.strokeStyle = 'rgba(255,255,255,0.04)';
        ctx.lineWidth = 1;
        const ySteps = 5;
        for (let i = 0; i <= ySteps; i++) {
            const elev = minElev + (elevRange / ySteps) * i;
            const y = yPos(elev);
            ctx.beginPath();
            ctx.moveTo(pad.left, y);
            ctx.lineTo(w - pad.right, y);
            ctx.stroke();
            ctx.fillStyle = '#64748b';
            ctx.font = '10px Inter, sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(`${elev.toFixed(0)}m`, pad.left - 6, y + 3);
        }

        const xSteps = 6;
        for (let i = 0; i <= xSteps; i++) {
            const d = (totalDist / xSteps) * i;
            const x = xPos(d);
            ctx.beginPath();
            ctx.moveTo(x, pad.top);
            ctx.lineTo(x, pad.top + drawH);
            ctx.stroke();
            ctx.fillStyle = '#64748b';
            ctx.font = '10px Inter, sans-serif';
            ctx.textAlign = 'center';
            const label = d >= 1000 ? `${(d / 1000).toFixed(1)}km` : `${d.toFixed(0)}m`;
            ctx.fillText(label, x, h - pad.bottom + 16);
        }

        // Earth curvature reference (flat line at 0 curvature)
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        for (let i = 0; i < profile.length; i++) {
            const x = xPos(profile[i].distance);
            const y = yPos(profile[i].terrain);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Terrain profile (with curvature)
        ctx.beginPath();
        ctx.moveTo(xPos(0), yPos(profile[0].terrain + profile[0].curvature));
        for (let i = 1; i < profile.length; i++) {
            ctx.lineTo(xPos(profile[i].distance), yPos(profile[i].terrain + profile[i].curvature));
        }
        ctx.lineTo(xPos(totalDist), yPos(minElev));
        ctx.lineTo(xPos(0), yPos(minElev));
        ctx.closePath();

        const terrainGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + drawH);
        terrainGrad.addColorStop(0, 'rgba(34, 120, 60, 0.6)');
        terrainGrad.addColorStop(1, 'rgba(34, 80, 40, 0.2)');
        ctx.fillStyle = terrainGrad;
        ctx.fill();

        // Terrain outline
        ctx.beginPath();
        ctx.moveTo(xPos(0), yPos(profile[0].terrain + profile[0].curvature));
        for (let i = 1; i < profile.length; i++) {
            ctx.lineTo(xPos(profile[i].distance), yPos(profile[i].terrain + profile[i].curvature));
        }
        ctx.strokeStyle = 'rgba(34, 197, 94, 0.6)';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Buildings
        for (let i = 0; i < profile.length; i++) {
            if (profile[i].inBuilding && profile[i].buildingHeight > 0) {
                const x = xPos(profile[i].distance);
                const baseY = yPos(profile[i].terrain + profile[i].curvature);
                const topY = yPos(profile[i].terrain + profile[i].curvature + profile[i].buildingHeight);
                const barWidth = Math.max(2, drawW / profile.length);

                ctx.fillStyle = 'rgba(156, 130, 100, 0.5)';
                ctx.fillRect(x - barWidth / 2, topY, barWidth, baseY - topY);
                ctx.strokeStyle = 'rgba(200, 170, 130, 0.6)';
                ctx.lineWidth = 0.5;
                ctx.strokeRect(x - barWidth / 2, topY, barWidth, baseY - topY);
            }
        }

        // Fresnel zone
        const losPoints = [];
        for (let i = 0; i < profile.length; i++) {
            const t = profile[i].distance / totalDist;
            const losH = analysis.txAltitude + (analysis.rxAltitude - analysis.txAltitude) * t;
            const fr = profile[i].fresnelRadius;
            losPoints.push({ x: xPos(profile[i].distance), losY: yPos(losH), fr: fr });
        }

        // Fresnel zone fill
        ctx.beginPath();
        for (let i = 0; i < losPoints.length; i++) {
            const x = losPoints[i].x;
            const y = yPos(analysis.txAltitude + (analysis.rxAltitude - analysis.txAltitude) * (i / (losPoints.length - 1)) + losPoints[i].fr);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        for (let i = losPoints.length - 1; i >= 0; i--) {
            const x = losPoints[i].x;
            const y = yPos(analysis.txAltitude + (analysis.rxAltitude - analysis.txAltitude) * (i / (losPoints.length - 1)) - losPoints[i].fr);
            ctx.lineTo(x, y);
        }
        ctx.closePath();

        const fresnelColor = analysis.worstClearancePct >= 60
            ? 'rgba(16, 185, 129, 0.12)'
            : analysis.worstClearancePct >= 0
                ? 'rgba(245, 158, 11, 0.12)'
                : 'rgba(239, 68, 68, 0.12)';
        ctx.fillStyle = fresnelColor;
        ctx.fill();

        // Fresnel zone outline
        ctx.strokeStyle = analysis.worstClearancePct >= 60
            ? 'rgba(16, 185, 129, 0.5)'
            : analysis.worstClearancePct >= 0
                ? 'rgba(245, 158, 11, 0.5)'
                : 'rgba(239, 68, 68, 0.5)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        // LOS line
        ctx.beginPath();
        ctx.moveTo(xPos(0), yPos(analysis.txAltitude));
        ctx.lineTo(xPos(totalDist), yPos(analysis.rxAltitude));
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Antenna markers
        // TX
        const txBaseY = yPos(profile[0].terrain + profile[0].curvature);
        const txTopY = yPos(analysis.txAltitude);
        ctx.beginPath();
        ctx.moveTo(xPos(0), txBaseY);
        ctx.lineTo(xPos(0), txTopY);
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(xPos(0), txTopY, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#f59e0b';
        ctx.fill();

        // RX
        const rxBaseY = yPos(profile[profile.length - 1].terrain + profile[profile.length - 1].curvature);
        const rxTopY = yPos(analysis.rxAltitude);
        ctx.beginPath();
        ctx.moveTo(xPos(totalDist), rxBaseY);
        ctx.lineTo(xPos(totalDist), rxTopY);
        ctx.strokeStyle = '#06b6d4';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(xPos(totalDist), rxTopY, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#06b6d4';
        ctx.fill();

        // Labels
        ctx.font = 'bold 9px Inter, sans-serif';
        ctx.fillStyle = '#f59e0b';
        ctx.textAlign = 'center';
        ctx.fillText('Tx', xPos(0), txTopY - 10);
        ctx.fillStyle = '#06b6d4';
        ctx.fillText('Rx', xPos(totalDist), rxTopY - 10);

        // Legend
        const legendY = pad.top + 8;
        const legendX = pad.left + 10;
        ctx.font = '9px Inter, sans-serif';

        ctx.fillStyle = 'rgba(34, 197, 94, 0.8)';
        ctx.fillRect(legendX, legendY, 12, 8);
        ctx.fillStyle = '#94a3b8';
        ctx.textAlign = 'left';
        ctx.fillText('Terrain', legendX + 16, legendY + 7);

        ctx.fillStyle = 'rgba(156, 130, 100, 0.7)';
        ctx.fillRect(legendX + 70, legendY, 12, 8);
        ctx.fillStyle = '#94a3b8';
        ctx.fillText('Buildings', legendX + 86, legendY + 7);

        ctx.fillStyle = fresnelColor.replace('0.12', '0.5');
        ctx.fillRect(legendX + 148, legendY, 12, 8);
        ctx.fillStyle = '#94a3b8';
        ctx.fillText('1st Fresnel Zone', legendX + 164, legendY + 7);

        // Store rendering params for tooltip
        canvas._renderParams = { pad, drawW, drawH, totalDist, minElev, elevRange, profile, analysis };
    }

    // --- Profile Tooltip ---

    function setupProfileTooltip(canvas) {
        const tooltip = document.getElementById('profile-tooltip');

        canvas.addEventListener('mousemove', (e) => {
            const params = canvas._renderParams;
            if (!params) return;

            const rect = canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;

            const { pad, drawW, totalDist, profile, analysis } = params;

            if (x < pad.left || x > pad.left + drawW) {
                tooltip.classList.add('hidden');
                return;
            }

            const d = ((x - pad.left) / drawW) * totalDist;
            const idx = Math.round((d / totalDist) * (profile.length - 1));
            if (idx < 0 || idx >= profile.length) {
                tooltip.classList.add('hidden');
                return;
            }

            const p = profile[idx];
            const t = p.distance / totalDist;
            const losH = analysis.txAltitude + (analysis.rxAltitude - analysis.txAltitude) * t;
            const clearance = losH - p.effectiveHeight;
            const clearancePct = p.fresnelRadius > 0 ? (clearance / p.fresnelRadius * 100).toFixed(0) : '--';

            tooltip.innerHTML = `
                <strong>Distance:</strong> ${p.distance >= 1000 ? (p.distance / 1000).toFixed(2) + ' km' : p.distance.toFixed(0) + ' m'}<br>
                <strong>Terrain:</strong> ${p.terrain.toFixed(1)} m ASL<br>
                ${p.inBuilding ? `<strong>Building:</strong> +${p.buildingHeight.toFixed(1)} m<br>` : ''}
                <strong>LoS Height:</strong> ${losH.toFixed(1)} m<br>
                <strong>Clearance:</strong> ${clearance.toFixed(1)} m (${clearancePct}% F1)
            `;

            tooltip.classList.remove('hidden');
            tooltip.style.left = `${e.clientX - rect.left + 12}px`;
            tooltip.style.top = `${e.clientY - rect.top - 10}px`;
        });

        canvas.addEventListener('mouseleave', () => {
            tooltip.classList.add('hidden');
        });
    }

    // --- Map Setup ---

    function initMap() {
        state.map = L.map('map', {
            center: CONFIG.defaultCenter,
            zoom: CONFIG.defaultZoom,
            zoomControl: true,
        });

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
            maxZoom: 19,
        }).addTo(state.map);

        state.map.on('click', onMapClick);
    }

    function createMarkerIcon(type) {
        const color = type === 'tx' ? '#f59e0b' : '#06b6d4';
        const label = type === 'tx' ? 'Tx' : 'Rx';
        return L.divIcon({
            className: 'custom-marker',
            html: `
                <div class="marker-label">${label}</div>
                <svg width="28" height="36" viewBox="0 0 28 36">
                    <path d="M14 0 C6.268 0 0 6.268 0 14 C0 24.5 14 36 14 36 C14 36 28 24.5 28 14 C28 6.268 21.732 0 14 0Z" fill="${color}"/>
                    <circle cx="14" cy="14" r="6" fill="white" opacity="0.9"/>
                </svg>
            `,
            iconSize: [28, 36],
            iconAnchor: [14, 36],
        });
    }

    function onMapClick(e) {
        const { lat, lng } = e.latlng;

        // Hide instructions once user starts interacting
        const instrCard = document.querySelector('.instructions-card');
        if (instrCard) instrCard.style.display = 'none';

        if (state.placingMode === 'tx') {
            if (state.txMarker) state.map.removeLayer(state.txMarker);
            state.txMarker = L.marker([lat, lng], {
                icon: createMarkerIcon('tx'),
                draggable: true,
            }).addTo(state.map);
            state.txMarker.on('dragend', () => {
                state.txPoint = state.txMarker.getLatLng();
                updateCoordDisplay('tx', state.txPoint);
                updatePathLine();
            });
            state.txPoint = { lat, lng };
            updateCoordDisplay('tx', state.txPoint);
            state.placingMode = 'rx';
        } else if (state.placingMode === 'rx') {
            if (state.rxMarker) state.map.removeLayer(state.rxMarker);
            state.rxMarker = L.marker([lat, lng], {
                icon: createMarkerIcon('rx'),
                draggable: true,
            }).addTo(state.map);
            state.rxMarker.on('dragend', () => {
                state.rxPoint = state.rxMarker.getLatLng();
                updateCoordDisplay('rx', state.rxPoint);
                updatePathLine();
            });
            state.rxPoint = { lat, lng };
            updateCoordDisplay('rx', state.rxPoint);
            state.placingMode = 'done';
            document.getElementById('btn-calculate').disabled = false;
        }

        updatePathLine();
    }

    function updateCoordDisplay(type, point) {
        const el = document.getElementById(`${type}-coords`);
        el.textContent = `${point.lat.toFixed(6)}, ${point.lng.toFixed(6)}`;
        el.classList.add('active');
    }

    function updatePathLine() {
        if (state.pathLine) {
            state.map.removeLayer(state.pathLine);
            state.pathLine = null;
        }
        if (state.fresnelPolygon) {
            state.map.removeLayer(state.fresnelPolygon);
            state.fresnelPolygon = null;
        }

        if (state.txPoint && state.rxPoint) {
            state.pathLine = L.polyline(
                [[state.txPoint.lat, state.txPoint.lng], [state.rxPoint.lat, state.rxPoint.lng]],
                { color: '#ffffff', weight: 2, opacity: 0.7, dashArray: '6, 6' }
            ).addTo(state.map);
        }
    }

    function drawFresnelOnMap(profile, analysis) {
        if (state.fresnelPolygon) {
            state.map.removeLayer(state.fresnelPolygon);
        }

        const totalDist = analysis.totalDistance;
        const points = [];
        const reversePoints = [];

        for (let i = 0; i < profile.length; i++) {
            const p = profile[i];
            const fr = p.fresnelRadius;
            // Calculate perpendicular offset in lat/lng
            const bearing = Math.atan2(
                state.rxPoint.lng - state.txPoint.lng,
                state.rxPoint.lat - state.txPoint.lat
            );
            const perpBearing = bearing + Math.PI / 2;
            const offsetLat = (fr / CONFIG.earthRadius) * toDeg(1) * Math.cos(perpBearing);
            const offsetLng = (fr / CONFIG.earthRadius) * toDeg(1) * Math.sin(perpBearing) / Math.cos(toRad(p.lat));

            points.push([p.lat + offsetLat, p.lng + offsetLng]);
            reversePoints.unshift([p.lat - offsetLat, p.lng - offsetLng]);
        }

        const fresnelCoords = [...points, ...reversePoints];
        const color = analysis.worstClearancePct >= 60 ? '#10b981'
            : analysis.worstClearancePct >= 0 ? '#f59e0b' : '#ef4444';

        state.fresnelPolygon = L.polygon(fresnelCoords, {
            color: color,
            weight: 1,
            fillColor: color,
            fillOpacity: 0.15,
            opacity: 0.5,
        }).addTo(state.map);
    }

    // --- Main Calculation ---

    async function calculate() {
        if (!state.txPoint || !state.rxPoint) return;

        const loading = document.getElementById('loading');
        const loadingStatus = document.getElementById('loading-status');
        loading.classList.remove('hidden');

        try {
            // Get parameters
            const txHeight = parseFloat(document.getElementById('tx-height').value) || 0;
            const rxHeight = parseFloat(document.getElementById('rx-height').value) || 0;
            const txPower = parseFloat(document.getElementById('tx-power').value) || 14;
            const txGain = parseFloat(document.getElementById('tx-gain').value) || 0;
            const rxGain = parseFloat(document.getElementById('rx-gain').value) || 0;
            const rxSensitivity = parseFloat(document.getElementById('rx-sensitivity').value) || -132;

            const totalDistance = haversineDistance(
                state.txPoint.lat, state.txPoint.lng,
                state.rxPoint.lat, state.rxPoint.lng
            );

            // Sample points along path
            const points = interpolatePoints(state.txPoint, state.rxPoint, CONFIG.pathSamples);

            // Fetch elevation
            loadingStatus.textContent = 'Fetching elevation data...';
            const elevations = await fetchElevation(points);

            // Fetch buildings
            loadingStatus.textContent = 'Querying building data...';
            const buildings = await fetchBuildings(points, totalDistance);

            // Calculate profile
            loadingStatus.textContent = 'Calculating Fresnel zone...';
            const profile = calculateProfile(points, elevations, buildings, totalDistance, state.txPoint, state.rxPoint);
            state.profileData = profile;
            state.buildingData = buildings;

            // Analyze link
            const analysis = analyzeLink(profile, txHeight, rxHeight, txPower, txGain, rxGain, rxSensitivity);

            // Update UI
            displayResults(analysis, totalDistance);

            // Render profile chart
            const profileContainer = document.getElementById('profile-container');
            profileContainer.classList.remove('hidden');
            state.map.invalidateSize();
            // Small delay to ensure container is visible before rendering
            await new Promise(r => setTimeout(r, 50));
            const canvas = document.getElementById('profile-canvas');
            renderProfile(canvas, profile, analysis);

            // Draw Fresnel zone on map
            drawFresnelOnMap(profile, analysis);

            // Update path line style
            if (state.pathLine) {
                state.map.removeLayer(state.pathLine);
            }
            const lineColor = analysis.worstClearancePct >= 60 ? '#10b981'
                : analysis.worstClearancePct >= 0 ? '#f59e0b' : '#ef4444';
            state.pathLine = L.polyline(
                [[state.txPoint.lat, state.txPoint.lng], [state.rxPoint.lat, state.rxPoint.lng]],
                { color: lineColor, weight: 3, opacity: 0.9 }
            ).addTo(state.map);

        } catch (error) {
            console.error('Calculation error:', error);
            alert('Error during calculation: ' + error.message);
        } finally {
            loading.classList.add('hidden');
        }
    }

    function displayResults(analysis) {
        const results = document.getElementById('results');
        results.classList.remove('hidden');

        // Distance
        const distStr = analysis.totalDistance >= 1000
            ? `${(analysis.totalDistance / 1000).toFixed(2)} km`
            : `${analysis.totalDistance.toFixed(0)} m`;
        document.getElementById('res-distance').textContent = distStr;

        // FSPL
        document.getElementById('res-fspl').textContent = `${analysis.fspl.toFixed(1)} dB`;

        // Fresnel radius
        document.getElementById('res-fresnel-r').textContent = `${analysis.maxFresnelR.toFixed(1)} m`;

        // Clearance
        const clearanceEl = document.getElementById('res-clearance');
        clearanceEl.textContent = `${analysis.worstClearancePct.toFixed(0)}% F1`;
        clearanceEl.style.color = analysis.worstClearancePct >= 60 ? '#10b981'
            : analysis.worstClearancePct >= 0 ? '#f59e0b' : '#ef4444';

        // Obstruction loss
        const obsText = analysis.obstructionCount > 0
            ? `${analysis.obstructionLoss.toFixed(1)} dB (${analysis.obstructionCount} obst.)`
            : `${analysis.obstructionLoss.toFixed(1)} dB`;
        document.getElementById('res-obstruction').textContent = obsText;

        // Received power
        const rxPowerEl = document.getElementById('res-rx-power');
        rxPowerEl.textContent = `${analysis.receivedPower.toFixed(1)} dBm`;

        // Link margin
        const marginEl = document.getElementById('res-margin');
        marginEl.textContent = `${analysis.linkMargin.toFixed(1)} dB`;
        marginEl.style.color = analysis.linkMargin >= 10 ? '#10b981'
            : analysis.linkMargin >= 3 ? '#f59e0b' : '#ef4444';

        // Signal quality badge
        const signalCard = document.getElementById('signal-card');
        signalCard.className = `card signal-card ${analysis.qualityClass}`;

        const icons = {
            'signal-excellent': '📶',
            'signal-good': '📶',
            'signal-fair': '⚠️',
            'signal-poor': '⛔',
            'signal-none': '❌',
        };

        document.getElementById('signal-icon').textContent = icons[analysis.qualityClass] || '📶';
        document.getElementById('signal-quality').textContent = analysis.quality;
        document.getElementById('signal-desc').textContent = analysis.description;
        document.getElementById('signal-details').textContent = analysis.recommendations;
    }

    // --- Reset ---

    function reset() {
        if (state.txMarker) { state.map.removeLayer(state.txMarker); state.txMarker = null; }
        if (state.rxMarker) { state.map.removeLayer(state.rxMarker); state.rxMarker = null; }
        if (state.pathLine) { state.map.removeLayer(state.pathLine); state.pathLine = null; }
        if (state.fresnelPolygon) { state.map.removeLayer(state.fresnelPolygon); state.fresnelPolygon = null; }

        state.txPoint = null;
        state.rxPoint = null;
        state.placingMode = 'tx';
        state.profileData = null;

        document.getElementById('tx-coords').textContent = 'Click map to place';
        document.getElementById('tx-coords').classList.remove('active');
        document.getElementById('rx-coords').textContent = 'Click map to place';
        document.getElementById('rx-coords').classList.remove('active');
        document.getElementById('btn-calculate').disabled = true;
        document.getElementById('results').classList.add('hidden');
        document.getElementById('profile-container').classList.add('hidden');

        // Restore instructions card
        const instrCard = document.querySelector('.instructions-card');
        if (instrCard) instrCard.style.display = '';
    }

    // --- Window Resize ---

    function onResize() {
        if (state.profileData && !document.getElementById('profile-container').classList.contains('hidden')) {
            const canvas = document.getElementById('profile-canvas');
            // Re-render with stored data (need analysis)
            const txHeight = parseFloat(document.getElementById('tx-height').value) || 0;
            const rxHeight = parseFloat(document.getElementById('rx-height').value) || 0;
            const txPower = parseFloat(document.getElementById('tx-power').value) || 14;
            const txGain = parseFloat(document.getElementById('tx-gain').value) || 0;
            const rxGain = parseFloat(document.getElementById('rx-gain').value) || 0;
            const rxSensitivity = parseFloat(document.getElementById('rx-sensitivity').value) || -132;
            const analysis = analyzeLink(state.profileData, txHeight, rxHeight, txPower, txGain, rxGain, rxSensitivity);
            renderProfile(canvas, state.profileData, analysis);
        }
    }

    // --- Initialize ---

    function init() {
        initMap();

        document.getElementById('btn-calculate').addEventListener('click', calculate);
        document.getElementById('btn-reset').addEventListener('click', reset);
        document.getElementById('btn-close-profile').addEventListener('click', () => {
            document.getElementById('profile-container').classList.add('hidden');
            state.map.invalidateSize();
        });

        const canvas = document.getElementById('profile-canvas');
        setupProfileTooltip(canvas);

        window.addEventListener('resize', () => {
            state.map.invalidateSize();
            onResize();
        });
    }

    // Start app when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

})();
