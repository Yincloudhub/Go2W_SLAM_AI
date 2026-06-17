function create_map701_matlab_fig(dataJson, outFig, outPng, outMat)
%CREATE_MAP701_MATLAB_FIG Build the map_701 waypoint visualization figure.
%
% The script reads a JSON bundle generated from the pulled robot registry and
% PCD render metadata, then saves a MATLAB .fig that keeps the source model in
% fig.UserData for later analysis.

if nargin < 1 || strlength(string(dataJson)) == 0
    dataJson = "C:\Users\c\Desktop\map_701_matlab_visualization\map_701_matlab_data.json";
end
if nargin < 2 || strlength(string(outFig)) == 0
    outFig = "C:\Users\c\Desktop\map_701_waypoints_visualization.fig";
end
if nargin < 3 || strlength(string(outPng)) == 0
    outPng = "C:\Users\c\Desktop\map_701_waypoints_visualization.png";
end
if nargin < 4 || strlength(string(outMat)) == 0
    outMat = "C:\Users\c\Desktop\map_701_waypoints_visualization_data.mat";
end

model = jsondecode(fileread(dataJson));
fontName = "Microsoft YaHei";

fig = figure( ...
    "Name", "map_701 waypoint visualization", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "Units", "pixels", ...
    "Position", [80 60 1550 930]);
fig.UserData = model;

ax = axes("Parent", fig, "Position", [0.055 0.08 0.68 0.85]);
hold(ax, "on");
axis(ax, "equal");
grid(ax, "on");
box(ax, "on");
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.08;
ax.XMinorGrid = "on";
ax.YMinorGrid = "on";
ax.FontName = fontName;
ax.FontSize = 10;
xlabel(ax, "X (m)", "FontName", fontName);
ylabel(ax, "Y (m)", "FontName", fontName);

drawPcdBackground(ax, model);
drawRegions(ax, model, fontName);
drawManualAnnotations(ax, model, fontName);
drawConnectorEdges(ax, model, fontName);
drawOrigin(ax);
drawTransitionAnchors(ax, model, fontName);
drawNodes(ax, model, fontName);
drawLegend(ax, model);
setAxisFromContent(ax, model);

titleText = sprintf("%s / %s    frame=%s    nodes=%d", ...
    string(model.map_id), string(model.map_name), string(model.frame_id), numel(model.nodes));
title(ax, titleText, "FontName", fontName, "FontWeight", "bold", "Interpreter", "none");

buildSidePanel(fig, model, fontName);

drawnow;
savefig(fig, outFig);
try
    exportgraphics(fig, outPng, "Resolution", 220);
catch
    print(fig, outPng, "-dpng", "-r220");
end
save(outMat, "model");

fprintf("Saved FIG: %s\n", outFig);
fprintf("Saved PNG: %s\n", outPng);
fprintf("Saved MAT: %s\n", outMat);
end

function drawPcdBackground(ax, model)
if isfield(model, "floorplan_background_png")
    floorplanPath = string(model.floorplan_background_png);
    if isfile(floorplanPath) && isfield(model, "floorplan_meta") && isfield(model.floorplan_meta, "bounds_m")
        img = imread(floorplanPath);
        bounds = model.floorplan_meta.bounds_m;
        h = image(ax, ...
            "XData", [bounds.min_x bounds.max_x], ...
            "YData", [bounds.max_y bounds.min_y], ...
            "CData", img);
        h.AlphaData = 1.0;
        uistack(h, "bottom");
        set(ax, "YDir", "normal");
        return;
    end
end
pngPath = string(model.pcd_topdown_png);
if ~isfile(pngPath)
    warning("PCD topdown PNG not found: %s", pngPath);
    return;
end
img = imread(pngPath);
bounds = model.pcd_topdown_meta.bounds_m;
h = image(ax, ...
    "XData", [bounds.min_x bounds.max_x], ...
    "YData", [bounds.max_y bounds.min_y], ...
    "CData", img);
h.AlphaData = 0.42;
uistack(h, "bottom");
set(ax, "YDir", "normal");
end

function drawRegions(ax, model, fontName)
for idx = 1:numel(model.regions)
    r = model.regions(idx);
    x = [r.x_min r.x_max r.x_max r.x_min];
    y = [r.y_min r.y_min r.y_max r.y_max];
    c = reshape(double(r.color), 1, []);
    patch(ax, x, y, c, ...
        "FaceAlpha", 0.06, ...
        "EdgeAlpha", 0.55, ...
        "LineWidth", 1.2, ...
        "LineStyle", "--", ...
        "HandleVisibility", "off");
    text(ax, mean([r.x_min r.x_max]), r.y_max + 0.18, string(r.label), ...
        "HorizontalAlignment", "center", ...
        "VerticalAlignment", "bottom", ...
        "Interpreter", "none", ...
        "FontName", fontName, ...
        "FontSize", 10, ...
        "FontWeight", "bold", ...
        "Color", c * 0.72);
end
end

function drawManualAnnotations(ax, model, fontName)
if ~isfield(model, "manual_annotations") || ~isfield(model.manual_annotations, "features")
    return;
end
features = model.manual_annotations.features;
if isempty(features) || ~isstruct(features)
    return;
end
for idx = 1:numel(features)
    f = features(idx);
    if ~isfield(f, "points") || isempty(f.points) || ~isstruct(f.points)
        continue;
    end
    xs = arrayfun(@(p) double(p.x), f.points);
    ys = arrayfun(@(p) double(p.y), f.points);
    if numel(xs) < 2
        continue;
    end
    kind = string(getFieldOr(f, "kind", "wall"));
    label = string(getFieldOr(f, "label", ""));
    closed = logical(getFieldOr(f, "closed", false));
    switch kind
        case "large_obstruction"
            if closed && numel(xs) >= 3
                patch(ax, xs, ys, [0.55 0.57 0.59], ...
                    "FaceAlpha", 0.58, ...
                    "EdgeColor", [0.18 0.19 0.20], ...
                    "LineWidth", 1.4, ...
                    "HandleVisibility", "off");
            else
                plot(ax, xs, ys, "-", "Color", [0.55 0.57 0.59], "LineWidth", 5, "HandleVisibility", "off");
            end
        case "door_opening"
            plot(ax, xs, ys, "-", "Color", [0.03 0.48 0.27], "LineWidth", 5, "HandleVisibility", "off");
        case "no_go_boundary"
            plot(ax, xs, ys, "--", "Color", [0.72 0.22 0.14], "LineWidth", 3, "HandleVisibility", "off");
        otherwise
            plot(ax, xs, ys, "-", "Color", [0.12 0.13 0.14], "LineWidth", 6, "HandleVisibility", "off");
    end
    if strlength(label) > 0
        text(ax, mean(xs), mean(ys), label, ...
            "FontName", fontName, ...
            "FontSize", 9, ...
            "FontWeight", "bold", ...
            "Interpreter", "none", ...
            "Color", [0.12 0.13 0.14], ...
            "BackgroundColor", [1 1 1], ...
            "Margin", 3, ...
            "HandleVisibility", "off");
    end
end
end

function drawOrigin(ax)
plot(ax, [-0.28 0.28], [0 0], "k-", "LineWidth", 1.1, "HandleVisibility", "off");
plot(ax, [0 0], [-0.28 0.28], "k-", "LineWidth", 1.1, "HandleVisibility", "off");
text(ax, 0.08, 0.08, "origin", "Color", [0.05 0.05 0.05], ...
    "FontSize", 8, "Interpreter", "none", "HandleVisibility", "off");
quiver(ax, 0, 0, 0.85, 0, 0, "Color", [0.1 0.1 0.1], ...
    "LineWidth", 1.1, "MaxHeadSize", 0.7, "HandleVisibility", "off");
quiver(ax, 0, 0, 0, 0.85, 0, "Color", [0.1 0.1 0.1], ...
    "LineWidth", 1.1, "MaxHeadSize", 0.7, "HandleVisibility", "off");
text(ax, 0.92, 0.06, "+X", "FontSize", 8, "Interpreter", "none", "HandleVisibility", "off");
text(ax, 0.06, 0.92, "+Y", "FontSize", 8, "Interpreter", "none", "HandleVisibility", "off");
end

function value = getFieldOr(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function drawNodes(ax, model, fontName)
for idx = 1:numel(model.nodes)
    n = model.nodes(idx);
    primaryType = char(string(n.primary_type));
    marker = markerForType(primaryType);
    color = colorForType(model, primaryType);
    h = scatter(ax, n.x, n.y, 105, color, marker, ...
        "filled", ...
        "MarkerEdgeColor", [0.08 0.08 0.08], ...
        "LineWidth", 0.7, ...
        "HandleVisibility", "off");
    h.UserData = n;

    nodeTypes = toStringArray(n.types);
    if any(nodeTypes == "rotation_point") && primaryType ~= "rotation_point"
        scatter(ax, n.x, n.y, 165, colorForType(model, "rotation_point"), "o", ...
            "LineWidth", 1.6, ...
            "MarkerFaceColor", "none", ...
            "HandleVisibility", "off");
    end
    if any(nodeTypes == "corridor_endpoint") && primaryType ~= "corridor_endpoint"
        scatter(ax, n.x, n.y, 145, colorForType(model, "corridor_endpoint"), "s", ...
            "LineWidth", 1.4, ...
            "MarkerFaceColor", "none", ...
            "HandleVisibility", "off");
    end

    arrowLen = 0.58;
    quiver(ax, n.x, n.y, cos(n.yaw) * arrowLen, sin(n.yaw) * arrowLen, 0, ...
        "Color", [0.78 0.12 0.08], ...
        "LineWidth", 1.25, ...
        "MaxHeadSize", 0.95, ...
        "HandleVisibility", "off");

    label = sprintf("%d  %s\n%s\n(%.2f, %.2f) yaw %.1f deg", ...
        idx, string(n.name), string(n.node_id), n.x, n.y, n.yaw_deg);
    offset = labelOffset(idx);
    text(ax, n.x + offset(1), n.y + offset(2), label, ...
        "FontName", fontName, ...
        "FontSize", 9, ...
        "Interpreter", "none", ...
        "Color", [0.06 0.06 0.06], ...
        "BackgroundColor", [1 1 1], ...
        "Margin", 4, ...
        "EdgeColor", [0.82 0.84 0.86], ...
        "LineWidth", 0.5, ...
        "HandleVisibility", "off");
end
end

function drawTransitionAnchors(ax, model, fontName)
if ~isfield(model, "transition_anchors")
    return;
end
nodeIds = strings(1, numel(model.nodes));
for i = 1:numel(model.nodes)
    nodeIds(i) = string(model.nodes(i).node_id);
end
for idx = 1:numel(model.transition_anchors)
    t = model.transition_anchors(idx);
    if any(nodeIds == string(t.anchor_id))
        continue;
    end
    color = colorForType(model, "transition");
    scatter(ax, t.x, t.y, 130, color, "^", ...
        "filled", ...
        "MarkerEdgeColor", [0.06 0.06 0.06], ...
        "LineWidth", 0.8, ...
        "HandleVisibility", "off");
    quiver(ax, t.x, t.y, cos(t.yaw) * 0.65, sin(t.yaw) * 0.65, 0, ...
        "Color", color, ...
        "LineWidth", 1.3, ...
        "MaxHeadSize", 0.95, ...
        "LineStyle", "-", ...
        "HandleVisibility", "off");
    label = sprintf("%s\n-> %s", string(t.anchor_id), string(t.connects_to));
    text(ax, t.x + 0.18, t.y + 0.18, label, ...
        "FontName", fontName, ...
        "FontSize", 8.5, ...
        "Interpreter", "none", ...
        "Color", color * 0.75, ...
        "BackgroundColor", [1 1 1], ...
        "Margin", 4, ...
        "EdgeColor", [0.86 0.82 0.9], ...
        "HandleVisibility", "off");
end
end

function drawConnectorEdges(ax, model, fontName)
if ~isfield(model, "connector_edges")
    return;
end
for idx = 1:numel(model.connector_edges)
    edge = model.connector_edges(idx);
    fromNode = findNode(model, string(edge.from));
    toNode = findNode(model, string(edge.to));
    if isempty(fromNode) || isempty(toNode)
        continue;
    end
    color = colorForType(model, "transition");
    plot(ax, [fromNode.x toNode.x], [fromNode.y toNode.y], ...
        "LineStyle", "--", ...
        "LineWidth", 1.3, ...
        "Color", color, ...
        "HandleVisibility", "off");
    xm = (fromNode.x + toNode.x) / 2;
    ym = (fromNode.y + toNode.y) / 2;
    text(ax, xm, ym, string(edge.label), ...
        "FontName", fontName, ...
        "FontSize", 8.2, ...
        "Interpreter", "none", ...
        "Color", color * 0.72, ...
        "BackgroundColor", [1 1 1], ...
        "Margin", 3, ...
        "HorizontalAlignment", "center", ...
        "HandleVisibility", "off");
end
end

function drawLegend(ax, model)
patch(ax, nan, nan, [0.21 0.23 0.24], ...
    "FaceAlpha", 1.0, ...
    "EdgeColor", [0.16 0.17 0.18], ...
    "DisplayName", "wall / partition");
patch(ax, nan, nan, [0.53 0.55 0.57], ...
    "FaceAlpha", 1.0, ...
    "EdgeColor", [0.24 0.25 0.26], ...
    "DisplayName", "large obstruction");
scatter(ax, nan, nan, 95, colorForType(model, "attributed"), "o", "filled", ...
    "MarkerEdgeColor", [0.08 0.08 0.08], "DisplayName", "attributed");
scatter(ax, nan, nan, 95, colorForType(model, "corridor_endpoint"), "s", "filled", ...
    "MarkerEdgeColor", [0.08 0.08 0.08], "DisplayName", "corridor endpoint");
scatter(ax, nan, nan, 95, colorForType(model, "rotation_point"), "d", "filled", ...
    "MarkerEdgeColor", [0.08 0.08 0.08], "DisplayName", "rotation point");
scatter(ax, nan, nan, 105, colorForType(model, "transition"), "^", "filled", ...
    "MarkerEdgeColor", [0.08 0.08 0.08], "DisplayName", "transition anchor");
quiver(ax, nan, nan, nan, nan, 0, "Color", [0.78 0.12 0.08], ...
    "LineWidth", 1.25, "DisplayName", "yaw");
legend(ax, "Location", "southoutside", "Orientation", "horizontal");
end

function setAxisFromContent(ax, model)
xs = [];
ys = [];
for i = 1:numel(model.nodes)
    xs(end + 1) = model.nodes(i).x; %#ok<AGROW>
    ys(end + 1) = model.nodes(i).y; %#ok<AGROW>
end
for i = 1:numel(model.transition_anchors)
    xs(end + 1) = model.transition_anchors(i).x; %#ok<AGROW>
    ys(end + 1) = model.transition_anchors(i).y; %#ok<AGROW>
end
for i = 1:numel(model.regions)
    xs = [xs model.regions(i).x_min model.regions(i).x_max]; %#ok<AGROW>
    ys = [ys model.regions(i).y_min model.regions(i).y_max]; %#ok<AGROW>
end
if isempty(xs)
    return;
end
xlim(ax, [min(xs) - 1.4, max(xs) + 1.4]);
ylim(ax, [min(ys) - 1.2, max(ys) + 1.3]);
end

function buildSidePanel(fig, model, fontName)
panel = uipanel("Parent", fig, ...
    "Units", "normalized", ...
    "Position", [0.755 0.08 0.225 0.85], ...
    "Title", "map_701 data", ...
    "FontName", fontName, ...
    "FontSize", 11, ...
    "BackgroundColor", "w");

nodeData = cell(numel(model.nodes), 6);
for i = 1:numel(model.nodes)
    n = model.nodes(i);
    nodeData{i, 1} = char(string(n.node_id));
    nodeData{i, 2} = char(string(n.name));
    nodeData{i, 3} = char(strjoin(toStringArray(n.types), ", "));
    nodeData{i, 4} = n.x;
    nodeData{i, 5} = n.y;
    nodeData{i, 6} = n.yaw_deg;
end
uitable("Parent", panel, ...
    "Units", "normalized", ...
    "Position", [0.03 0.49 0.94 0.47], ...
    "Data", nodeData, ...
    "ColumnName", {"node_id", "name", "types", "x", "y", "yaw_deg"}, ...
    "ColumnWidth", {82, 105, 128, 58, 58, 64}, ...
    "FontName", fontName, ...
    "FontSize", 9);

sourceLines = [
    "Remote: " + string(model.source.remote_host)
    "Registry: " + string(model.source.remote_registry)
    "PCD: " + string(model.source.remote_pcd)
    "PCD SHA256: " + extractBetween(string(model.source.pcd_sha256), 1, 12) + "..."
    "Floorplan: " + string(model.floorplan_background_png)
    "Pulled files: " + string(model.source.pulled_registry)
    "Local data: " + "C:\Users\c\Desktop\map_701_matlab_visualization"
    "Note: read-only pull; robot files were not modified."
];
uicontrol("Parent", panel, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.04 0.04 0.92 0.39], ...
    "String", strjoin(sourceLines, newline), ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w", ...
    "FontName", fontName, ...
    "FontSize", 8.6);
end

function c = colorForType(model, typeName)
typeName = char(string(typeName));
if isfield(model.style.type_colors, typeName)
    c = reshape(double(model.style.type_colors.(typeName)), 1, []);
else
    c = [0.25 0.25 0.25];
end
end

function marker = markerForType(typeName)
switch char(string(typeName))
    case "attributed"
        marker = "o";
    case "corridor_endpoint"
        marker = "s";
    case "rotation_point"
        marker = "d";
    case "transition"
        marker = "^";
    otherwise
        marker = "o";
end
end

function node = findNode(model, nodeId)
node = [];
for i = 1:numel(model.nodes)
    if string(model.nodes(i).node_id) == nodeId
        node = model.nodes(i);
        return;
    end
end
end

function arr = toStringArray(value)
if isstring(value)
    arr = value(:)';
elseif ischar(value)
    arr = string({value});
elseif iscell(value)
    arr = strings(1, numel(value));
    for i = 1:numel(value)
        arr(i) = string(value{i});
    end
else
    arr = string(value);
end
arr = arr(strlength(arr) > 0);
end

function offset = labelOffset(idx)
offsets = [
    -1.80 0.68
    0.20 0.34
    0.22 -0.75
    -1.95 0.32
    -1.85 -0.72
    -0.20 0.44
    0.25 0.42
    0.24 0.38
];
offset = offsets(mod(idx - 1, size(offsets, 1)) + 1, :);
end
