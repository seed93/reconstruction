function lMask = RegionGrowing(dImg, dMaxDif, iSeed)
%REGIONGROWING A MEXed 2D/3D region growing algorithm.
%
%   lMASK = REGIONGROWING(dIMG, dMAXDIF, iSEED) Returns a binary mask
%   lMASK, the result of growing a region from the seed point iSEED.
%   The stoping critereon is fulfilled if no voxels in the region's
%   4-neighbourhood have an intensity difference smaller than dMAXDIF to
%   the region's mean intensity.
%
%   If the seed point is not supplied, a GUI lets you select it. If no
%   output is requested, the result of the region growing is visualized
%
% IMPORTANT NOTE: This Matlab function is a front-end for a fast mex
% function. Compile it by making the directiory containing this file your
% current Matlab working directory and typing
%
% >> mex RegionGrowing_mex.cpp
%
% in the Matlab console.
%
%
% Example (requires image processing toolbox):
%
%   load mri; % Gives variable D;
%   RegionGrowing(squeeze(D), 10, [1 1 1]); % <- segments the background
%
% Copyright 2013 Christian Wuerslin, University of Tuebingen and
% University of Stuttgart, Germany.
% Contact: christian.wuerslin@med.uni-tuebingen.de

% =========================================================================
% *** FUNCTION RegionGrowing
% ***
% *** See above for description.
% ***
% =========================================================================

dOPACITY = 0.6;

% -------------------------------------------------------------------------
% Parse the input arguments
if nargin < 2, error('At least two input arguments required!'); end
if ndims(dImg) > 3, error('Input image must be either 2D or 3D!'); end
dImg = double(dImg);
if ~isscalar(dMaxDif), error('Second input argument (MaxDif) must be a scalar!'); end
    
if nargin < 3
    iSeed = uint16(fGetSeed(dImg));
    if isempty(iSeed), return; end
else
    if numel(iSeed) ~= ndims(dImg), error('Invalid seed point! Must have ndims(dImg) elements!'); end
    iSeed = uint16(iSeed(:));
end
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% If the mex file has not been compiled yet, try to do so.
if exist('RegionGrowing_mex') ~= 3
    fprintf(1, 'Trying to compile mex file...');
    sCurrentPath = cd;
    sPath = fileparts(mfilename('fullpath'));
    cd(sPath)
    try
        mex([sPath, filesep, 'RegionGrowing_mex.cpp']);
        fprintf(1, 'done\n');
    catch
        error('Could not compile the mex file :(. Please try to do so manually!');
    end
    cd(sCurrentPath);
end
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% Start the region growing process by calling the mex function
lMask = RegionGrowing_mex(dImg, iSeed, dMaxDif);
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% If no output requested, visualize the result
if ~nargout
    dImg = dImg - min(dImg(:)); % Normalize the image
    dImg = dImg./max(dImg(:));
    dImg = permute(dImg, [1 2 4 3]); % Change to RGB-mode
    dImg = repmat(dImg, [1 1 3 1]);
    dMask = double(permute(lMask, [1 2 4 3])); 
    dMask = cat(3, dMask, zeros(size(dMask)), zeros(size(dMask))); % Make mask the red channel -> red overlay
    
    dImg = 1 - (1 - dImg).*(1 - dOPACITY.*dMask); % The 'screen' overlay mode

    % reduce montage size by selecting the interesting slices, only
    lSlices = squeeze(sum(sum(dMask(:,:,1,:), 1), 2) > 0 );
    figure, montage(dImg(:,:,:,lSlices));
end
% -------------------------------------------------------------------------

end
% =========================================================================
% *** END FUNCTION RegionGrowing
% =========================================================================



% =========================================================================
% *** FUNCTION fGetSeed
% ***
% *** A little GUI to select a seed
% ***
% =========================================================================
function iSeed = fGetSeed(dImg)
iSlice = 1;
dImg = dImg./max(dImg(:));
dImg = dImg - min(dImg(:));
iImg = uint8(dImg.*255);
try
    hF = figure(...
        'Position'             , [0 0 size(dImg, 2) size(dImg, 1)], ...
        'Units'                , 'pixels', ...
        'Color'                , 'k', ...
        'DockControls'         , 'off', ...
        'MenuBar'              , 'none', ...
        'Name'                 , 'Select Seed', ...
        'NumberTitle'          , 'off', ...
        'BusyAction'           , 'cancel', ...
        'Pointer'              , 'crosshair', ...
        'CloseRequestFcn'      , 'delete(gcbf)', ...
        'WindowButtonDownFcn'  , 'uiresume(gcbf)', ...
        'KeyPressFcn'          , @fKeyPressFcn, ...
        'WindowScrollWheelFcn' , @fWindowScrollWheelFcn);
catch
    hF = figure(...
        'Position'             , [0 0 size(dImg, 2) size(dImg, 1)], ...
        'Units'                , 'pixels', ...
        'Color'                , 'k', ...
        'DockControls'         , 'off', ...
        'MenuBar'              , 'none', ...
        'Name'                 , 'Select Seed', ...
        'NumberTitle'          , 'off', ...
        'BusyAction'           , 'cancel', ...
        'Pointer'              , 'crosshair', ...
        'CloseRequestFcn'      , 'delete(gcbf)', ...
        'WindowButtonDownFcn'  , 'uiresume(gcbf)', ...
        'KeyPressFcn'          , @fKeyPressFcn);
end

hA = axes(...
    'Parent'                , hF, ...
    'Position'              , [0 0 1 1]);
hI = image(iImg(:,:,1), ...
    'Parent'                , hA, ...
    'CDataMapping'          , 'scaled');

colormap(gray(256));
movegui('center');

uiwait;

if ~ishandle(hF)
    iSeed = [];
else
    iPos = uint16(get(hA, 'CurrentPoint'));
    iSeed = [iPos(1, 2) iPos(1, 1) iSlice];
    delete(hF);
end

    % ---------------------------------------------------------------------
    % * * NESTED FUNCTION fKeyPressFcn (nested in fGetSeed)
    % * *
    % * * Changes the active slice
    % ---------------------------------------------------------------------
    function fKeyPressFcn(hObject, eventdata)
        switch(eventdata.Key)
            case 'uparrow'
                iSlice = min([size(iImg, 3), iSlice + 1]);
                set(hI, 'CData', iImg(:,:,iSlice));
                
            case 'downarrow'
                iSlice = max([1, iSlice - 1]);
                set(hI, 'CData', iImg(:,:,iSlice));
                
        end
    end
    % ---------------------------------------------------------------------
    % * * END OF NESTED FUNCTION fKeyPressFcn (nested in fGetSeed)
    % ---------------------------------------------------------------------
    
    
    % ---------------------------------------------------------------------
    % * * NESTED FUNCTION fWindowScrollWheelFcn (nested in fGetSeed)
    % * *
    % * * Changes the active slice
    % ---------------------------------------------------------------------
    function fWindowScrollWheelFcn(hObject, eventdata)
        iSlice = min([size(iImg, 3), iSlice + eventdata.VerticalScrollCount]);
        iSlice = max([1, iSlice]);
        set(hI, 'CData', iImg(:,:,iSlice));
    end
    % ---------------------------------------------------------------------
    % * * END OF NESTED FUNCTION fWindowScrollWheelFcn (nested in fGetSeed)
    % ---------------------------------------------------------------------

end
% =========================================================================
% *** END FUNCTION fGetSeed (and its nested function)
% =========================================================================