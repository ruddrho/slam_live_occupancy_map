function combineProjectVideos(cfg)
%COMBINEPROJECTVIDEOS Fast side-by-side video composition.
%
% Left: A* search and route animation
% Right: Hybrid A* + DWA navigation
%
% Performance improvements:
% - source videos are read sequentially instead of repeatedly seeking,
% - the title band is rendered once and reused,
% - the shorter video holds its final frame.

astarPath = fullfile( ...
    cfg.output.resultsFolder, ...
    'astar_route_animation.avi');

navigationPath = fullfile( ...
    cfg.output.resultsFolder, ...
    'hybrid_astar_dwa_navigation.avi');

combinedPath = fullfile( ...
    cfg.output.resultsFolder, ...
    'combined_astar_navigation.avi');

if ~exist(astarPath,'file') || ...
        ~exist(navigationPath,'file')
    warning(['Combined video was skipped because one or both ' ...
        'source videos are missing.']);
    return;
end

leftReader = VideoReader(astarPath);
rightReader = VideoReader(navigationPath);

outputFrameRate = cfg.output.combinedVideo.frameRate;
panelHeight = cfg.output.combinedVideo.panelHeight;
panelWidth = cfg.output.combinedVideo.panelWidth;
titleBandHeight = cfg.output.combinedVideo.titleBandHeight;
gapWidth = cfg.output.combinedVideo.gapWidth;
backgroundValue = cfg.output.combinedVideo.backgroundValue;

outputHeight = panelHeight+titleBandHeight;
outputWidth = 2*panelWidth+gapWidth;

titleBand = createLabelBandLocal( ...
    outputWidth,titleBandHeight, ...
    panelWidth,gapWidth,backgroundValue);

videoObject = VideoWriter( ...
    combinedPath,'Motion JPEG AVI');

videoObject.FrameRate = outputFrameRate;
open(videoObject);

leftFrame = [];
rightFrame = [];

if hasFrame(leftReader)
    leftFrame = readFrame(leftReader);
end

if hasFrame(rightReader)
    rightFrame = readFrame(rightReader);
end

leftNextTime = 1/max(leftReader.FrameRate,1);
rightNextTime = 1/max(rightReader.FrameRate,1);

maximumDuration = max( ...
    leftReader.Duration,rightReader.Duration);

numberOfFrames = max(1,ceil( ...
    maximumDuration*outputFrameRate));

try
    for outputIndex = 0:numberOfFrames-1
        currentTime = outputIndex/outputFrameRate;

        while hasFrame(leftReader) && ...
                leftNextTime <= currentTime+1e-9
            leftFrame = readFrame(leftReader);
            leftNextTime = leftNextTime+ ...
                1/max(leftReader.FrameRate,1);
        end

        while hasFrame(rightReader) && ...
                rightNextTime <= currentTime+1e-9
            rightFrame = readFrame(rightReader);
            rightNextTime = rightNextTime+ ...
                1/max(rightReader.FrameRate,1);
        end

        if isempty(leftFrame) || isempty(rightFrame)
            continue;
        end

        leftPanel = fitPanelLocal( ...
            leftFrame,panelHeight,panelWidth, ...
            backgroundValue);

        rightPanel = fitPanelLocal( ...
            rightFrame,panelHeight,panelWidth, ...
            backgroundValue);

        outputFrame = uint8(backgroundValue*ones( ...
            outputHeight,outputWidth,3));

        outputFrame(1:titleBandHeight,:,:) = titleBand;

        outputFrame( ...
            titleBandHeight+1:end, ...
            1:panelWidth,:) = leftPanel;

        outputFrame( ...
            titleBandHeight+1:end, ...
            panelWidth+gapWidth+1:end,:) = rightPanel;

        outputFrame(:,panelWidth+(1:gapWidth),:) = ...
            backgroundValue;

        writeVideo(videoObject,outputFrame);
    end

    close(videoObject);
catch errorInformation
    try
        close(videoObject);
    catch
    end
    rethrow(errorInformation);
end
end

function labelBand = createLabelBandLocal( ...
    outputWidth,titleBandHeight, ...
    panelWidth,gapWidth,backgroundValue)
%CREATELABELBANDLOCAL Render the titles once.

titleFigure = figure( ...
    'Visible','off', ...
    'Color','w', ...
    'Position',[50 50 outputWidth titleBandHeight+15], ...
    'NumberTitle','off');

titleAxes = axes( ...
    'Parent',titleFigure, ...
    'Position',[0 0 1 1], ...
    'XLim',[0 1],'YLim',[0 1], ...
    'XTick',[],'YTick',[]);

axis(titleAxes,'off');
hold(titleAxes,'on');

leftCentre = ...
    (0.5*panelWidth)/outputWidth;

rightCentre = ...
    (panelWidth+gapWidth+0.5*panelWidth)/outputWidth;

text(titleAxes,leftCentre,0.52, ...
    'A* Search + Route Animation', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontWeight','bold', ...
    'FontSize',13, ...
    'Color',[0.03 0.25 0.70], ...
    'Interpreter','none');

text(titleAxes,rightCentre,0.52, ...
    'Hybrid A* + DWA Navigation', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontWeight','bold', ...
    'FontSize',13, ...
    'Color',[0.00 0.45 0.10], ...
    'Interpreter','none');

temporaryImage = [tempname '.png'];

drawnow;
print(titleFigure,temporaryImage,'-dpng','-r90');

labelImage = imread(temporaryImage);

if exist(temporaryImage,'file')
    delete(temporaryImage);
end

close(titleFigure);

if ndims(labelImage) == 2
    labelImage = repmat(labelImage,[1 1 3]);
elseif size(labelImage,3) > 3
    labelImage = labelImage(:,:,1:3);
end

labelBand = fitPanelLocal( ...
    labelImage,titleBandHeight,outputWidth, ...
    backgroundValue);

labelBand(end,:,:) = 210;
end

function panel = fitPanelLocal( ...
    inputFrame,targetHeight,targetWidth,backgroundValue)

if isempty(inputFrame)
    panel = uint8(backgroundValue*ones( ...
        targetHeight,targetWidth,3));
    return;
end

inputHeight = size(inputFrame,1);
inputWidth = size(inputFrame,2);

scale = min( ...
    targetWidth/inputWidth, ...
    targetHeight/inputHeight);

newWidth = max(1,round(scale*inputWidth));
newHeight = max(1,round(scale*inputHeight));

resizedFrame = resizeFrameLocal( ...
    inputFrame,newHeight,newWidth);

panel = uint8(backgroundValue*ones( ...
    targetHeight,targetWidth,3));

rowStart = floor((targetHeight-newHeight)/2)+1;
columnStart = floor((targetWidth-newWidth)/2)+1;

panel( ...
    rowStart:rowStart+newHeight-1, ...
    columnStart:columnStart+newWidth-1,:) = ...
    resizedFrame;
end

function outputFrame = resizeFrameLocal( ...
    inputFrame,newHeight,newWidth)
%RESIZEFRAMELOCAL Toolbox-free bilinear resizing.

inputFrame = double(inputFrame);

inputHeight = size(inputFrame,1);
inputWidth = size(inputFrame,2);

if inputHeight == newHeight && inputWidth == newWidth
    outputFrame = uint8(inputFrame);
    return;
end

sourceX = linspace(1,inputWidth,newWidth);
sourceY = linspace(1,inputHeight,newHeight);

[queryX,queryY] = meshgrid(sourceX,sourceY);

baseX = floor(queryX);
baseY = floor(queryY);

baseX = max(1,min(baseX,max(inputWidth-1,1)));
baseY = max(1,min(baseY,max(inputHeight-1,1)));

nextX = min(baseX+1,inputWidth);
nextY = min(baseY+1,inputHeight);

deltaX = queryX-baseX;
deltaY = queryY-baseY;

outputFrame = zeros(newHeight,newWidth,3);

for channel = 1:3
    channelImage = inputFrame(:,:,channel);

    index11 = sub2ind( ...
        [inputHeight inputWidth],baseY,baseX);
    index12 = sub2ind( ...
        [inputHeight inputWidth],baseY,nextX);
    index21 = sub2ind( ...
        [inputHeight inputWidth],nextY,baseX);
    index22 = sub2ind( ...
        [inputHeight inputWidth],nextY,nextX);

    value11 = channelImage(index11);
    value12 = channelImage(index12);
    value21 = channelImage(index21);
    value22 = channelImage(index22);

    topValue = value11.*(1-deltaX)+ ...
        value12.*deltaX;

    bottomValue = value21.*(1-deltaX)+ ...
        value22.*deltaX;

    outputFrame(:,:,channel) = ...
        topValue.*(1-deltaY)+ ...
        bottomValue.*deltaY;
end

outputFrame = uint8(min(max( ...
    round(outputFrame),0),255));
end
