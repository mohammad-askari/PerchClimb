function [tableData,tableRaw] = extractMotive(varargin)
% EXTRACTMOTIVE reads data from a set of '.csv' files exported from the OptiTrack Motive
%   software. The data are processed and grouped as rigid body markers, .................
%   Note that the script is compatible with the data extracted from Motive-Tracker module
%   and is not intended to work with Motive-Body data.

%   Author:  Mohammad Askari
%   E-mail:  mohammad.askari@epfl.ch
%   Created: January 2022, using MATLAB_R2021b
%   Copyright 2022 - Laboratory of Intelligent Systems, EPFL.

    % Initialize basic common settings for data extraction
    output = 'struct';          % raw data format as: 'array' or 'struct'
    wpassfilter = 0;            % speed profiles estimation lowpass filter: [0 1]
    
    % Define desired coordinate reference frame mapping
    %                  OptiTrack Rigid Body  -->  Desired Body Frame
    mapping = struct(           'X'           ,           '+X'           ,...
                                'Y'           ,           '-Z'           ,...
                                'Z'           ,           '+Y'           );
    
    %% DATA EXTRACTION
    
    % Define the validation functions and input filetypes for the parser
    filesDefaultVal  = '';
    path = cd;
    if nargin == 1, files = varargin{1};
    else, files = filesDefaultVal;
    end
    
    % If no file is specified for data extraction, prompt user for file selection
    if isequal(files,filesDefaultVal)
        [files,path] = uigetfile({'*.csv;*.xlsx;*.xls;*.txt',...
                                  'Data Files (*.csv,*.xlsx,*.txt)'},...
                                  'Select Motion Capture System Data File(s)',...
                                  'MultiSelect','on');
        % Check if at least a valid file is selected
        if isequal(files,0), return; end
        if ischar(files), files = {files}; end
    end
    
    if ischar(files), files = {files}; end
    
    % Loop through all selected files
    for i = length(files):-1:1
        file = files{i};
        % Extract the experimental data and info
        [data,info] = extractData(path,file,output);
        
        % Remove rigid body missing data points and extract the robot data
        [info,t,x,y,z,rx,ry,rz] = robotData(file,data,info,mapping);
        
        % Apply coordinate frame correction and unwraping for roll, pitch, yaw
        sequence = info.rot;
        [phi,theta,psi] = eulerAngles(rx,ry,rz,sequence);
        
        % Estimate the linear and angular velocities along body axes frame
        fps = info.fps;
        [u,v,w,p,q,r] = estimateSpeeds(t,x,y,z,phi,theta,psi,fps,wpassfilter);
        
        % Convert the angles and angular rates to degrees before saving
        phi   = rad2deg(phi);
        theta = rad2deg(theta);
        psi   = rad2deg(psi);
        p     = rad2deg(p);
        q     = rad2deg(q);
        r     = rad2deg(r);
        
        % Populate a struct array with the experimental data set
        Time            = t;
        Position        = table(x,y,z);
        Orientation     = table(phi,theta,psi);
        LinearVelocity  = table(u,v,w);
        AngularVelocity = table(p,q,r);
        
        structData(i) = struct('Date',info.date,'Robot',info.robot,'Time',Time,...
                               'Position',Position,'Orientation',Orientation,...
                               'LinearVelocity' ,LinearVelocity,...
                               'AngularVelocity',AngularVelocity);
        
        fields = fieldnames(data);
        for k = 1:length(fields)
            structRaw(i).(fields{k}) = data.(fields{k});
        end
    end
    
    % Convert the full data set to a table and save it
    tableData = struct2table(structData,'AsArray',true);
    tableData.Robot = categorical(tableData.Robot);
    tableRaw  = struct2table(structRaw,'AsArray',true);
    
    % Save data if needed
%     filename = [path,'Experimental_Data'];
%     save(filename,'tableData');

end


%% SUB-FUNCTIONS

function varargout = extractData(path,file,output)
    % Create absolute path to the file and get its extension
    filepath  = [path,file];
    [~,~,ext] = fileparts(filepath);
    
    % Verify the output format and file type, then read the numeric data portion
    out = any(strcmpi(output,{'array','struct'}));
    ext = any(strcmpi(ext,{'.csv','.xlsx','.xls','.txt'}));
    assert(out,'Output format must be set to either ''array'' or ''struct''.');
    assert(ext,['Error reading: ',file,'\nPlease select a valid data file.']);
        
    % Read the numeric data portion
    data = readmatrix(filepath);
    
    % Delete the first few headerlines/rows if bad data exists
    line = find(diff(data(:,1)) == 1,1);
    data = data(line:end,:);
    
    % Remove the columns that are full of nans
    cols = all(isnan(data));
    data(:,cols) = [];
    
    % Extract general information from the first headerline if necessary
    if nargout > 1 || strcmpi(output,'struct')
        [info,flag] = extractInfo(path,file);
    end
    
    % Construct a struct data output using headerlines information if required
    if strcmpi(output,'struct') && flag
            % Read the first few headerlines containing possible information
            paramsData = readcell(filepath,'Range','3:7');
            paramsData(cellfun(@(x) all(ismissing(x)),paramsData)) = {''};
            
            % Remove unnecessary info and extract rigid bodies marker names
            k = 2;
            paramsData(:,1:k) = [];
            [fields,~,ic] = unique(paramsData(1,:),'stable');
            idx = (find(strcmpi(fields,'Rigid Body')) == ic);
            rigidBodies = unique(paramsData(2,idx),'stable');
            rigidBodiesMarkers = cell(1,size(paramsData,2));
            rigidBodiesMarkers(:) = {''};
            for i = 1:length(rigidBodies)
                str = extractAfter(paramsData(2,:),[rigidBodies{i},':']);
                rigidBodiesMarkers = join([rigidBodiesMarkers;str],'',1);
            end
            paramsData(3,:) = rigidBodiesMarkers;
            idx = ~ismissing(rigidBodiesMarkers);
            rigidBodiesMarkers(idx) = strcat(':',rigidBodiesMarkers(idx));
            paramsData(2,:) = strrep(paramsData(2,:),rigidBodiesMarkers,'');
            
            % Construct the data struct array with the parameters as fieldnames
            structData.Frame = data(:,1);
            structData.Time  = data(:,2);
            for i = 1:size(paramsData,2)
                fields = rmmissing(paramsData(:,i));
                fields = matlab.lang.makeValidName(fields);
                structData = setfield(structData,fields{:},data(:,i+k));
            end
            data = structData;
    end
    
    % Set the function outputs
    switch nargout
        case 1
            varargout{1} = data;
        case 2
            varargout{1} = data;
            varargout{2} = info;
        otherwise
            error('The maximum number of function outputs are two.');
    end
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------
function [info,flag] = extractInfo(path,file)
    % Set the default values in case headerline does not exist
    date = datetime('today');
    rot  = 'xyz';
    fps  = 240;
    unit = 'mm';
    
    % Extract general information from the first headerline if it exists
    filepath  = [path,file];
    exportSettings = readcell(filepath,'Range','1:1');
    hasHeaderlines = any(cellfun(@ischar,exportSettings),'all');

    if hasHeaderlines
        % Find required general export settings info
        date = findInfo(exportSettings,'Capture Start Time');
        rot  = findInfo(exportSettings,'Rotation Type');
        fps  = findInfo(exportSettings,'Export Frame Rate');
        unit = findInfo(exportSettings,'Length Units');

        date = regexpi(date,'\d*\-\d*\-\d*','match');
        date = datetime(date,'InputFormat','yyyy-MM-dd');
        unit = replace(unit,{'milli','centi','meters'},{'m','c','m'});
    else
        space = blanks(9);
        line1 = ['Warning: No header lines found in ''',file,'''.'];
        line2 = 'Toggle ''Header Information'' on in OptiTrack Motive during export.';
        line3 = 'The following default export settings are presumed:';
        disp(['[',8,line1,newline,space,line2,newline,space,line3,newline,']',8]);
    end
    
    % Set function outputs
    info = struct('date',date,'rot',rot,'fps',fps,'unit',unit);
    flag = hasHeaderlines;
    if ~flag
        disp(info);
    end
    
    % Subfunctions
    function value = findInfo(cellarray,name)
        ind = find(strcmp(cellarray,name));
        [row,col] = ind2sub(size(cellarray),ind);
        value = cellarray{row,col+1};
        if ischar(value), value = lower(value); end
    end
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------
function [info,t,x,y,z,rx,ry,rz] = robotData(file,data,info,mapping)
    % Extract gait, motor frequency, and footpads from the filename
    freq = regexpi(file,'\d*?\.?\d*(?=\s*Hz)','match');
    gait = deblank(extractBefore(file,char(freq)));
    freq = cellfun(@(x)str2double(x),freq);
    foot = contains(file,'footpad','IgnoreCase',true);
    
    % Get the coordinate reference frame mapping sequence and length scale unit
    unit    = info.unit;
    rotSeq  = info.rot;
    rotSeq  = cellstr(reshape(upper(rotSeq),[],1));
    mapping = orderfields(mapping);
    mapCell = struct2cell(mapping);
    [~,idx] = orderfields(mapping,rotSeq);
    
    posOpti = fieldnames(mapping);
    posSign = cellfun(@(x) str2double([x(1),'1']),mapCell);
    posBody = cellfun(@(x) lower({x(2)}),mapCell);
    rotOpti = posOpti(idx);
    rotSign = posSign(idx);
    rotSeq  = posBody(idx);
    rotBody = strcat('r',rotSeq);
    rotSeq  = strcat(rotSeq{:});
        
    % Gather robot's rigid body data with mapping correction from raw data
    if isstruct(data)
        cond = isfield(data,'RigidBody');
        assert(cond,['No ''Rigid Body'' data exists in: ',file,'.']);
        robot = fieldnames(data.RigidBody);
        if length(robot) > 1
            robot = robot(1);
            disp(['[',8,'Warning: Multiple rigid bodies found in the file. ',...
                  'The first robot: ',robot{1},' data are extracted.]',8]);
        end
        robot = char(robot);
        
        Position = data.RigidBody.(robot).Position;
        Rotation = data.RigidBody.(robot).Rotation;
        
        robotData.t = data.Time;        
        robotData.(rotBody{1}) = Rotation.(rotOpti{1}) * rotSign(1);
        robotData.(rotBody{2}) = Rotation.(rotOpti{2}) * rotSign(2);
        robotData.(rotBody{3}) = Rotation.(rotOpti{3}) * rotSign(3);
        robotData.(posBody{1}) = Position.(posOpti{1}) * posSign(1);
        robotData.(posBody{2}) = Position.(posOpti{2}) * posSign(2);
        robotData.(posBody{3}) = Position.(posOpti{3}) * posSign(3);
    else
        robot = '';
        robotData.t = data(:,2);
        robotData.(rotBody{1}) = data(:,3) * rotSign(1);
        robotData.(rotBody{2}) = data(:,4) * rotSign(2);
        robotData.(rotBody{3}) = data(:,5) * rotSign(3);
        robotData.(posBody{1}) = data(:,6) * posSign(1);
        robotData.(posBody{2}) = data(:,7) * posSign(2);
        robotData.(posBody{3}) = data(:,8) * posSign(3);
    end
    
    % Remove the missing frame data points
    dataArr = struct2array(robotData);
    [~,idx] = rmmissing(dataArr);
    robotData = structfun(@(x) x(~idx),robotData,'UniformOutput',false);
    
    % Normalize time and planar positions to start at zero
    robotData.t = zeroing(robotData.t);
    robotData.(posBody{1}) = zeroing(robotData.(posBody{1}));
    robotData.(posBody{3}) = zeroing(robotData.(posBody{3}));
    
    % Set the function outputs in desired units with missing data points removed
    t  = robotData.t;                 % [s]
    x  = conv2mm(robotData.x,unit);   % [mm]
    y  = conv2mm(robotData.y,unit);   % [mm]
    z  = conv2mm(robotData.z,unit);   % [mm]
    rx = deg2rad(robotData.rx);       % [rad]
    ry = deg2rad(robotData.ry);       % [rad]
    rz = deg2rad(robotData.rz);       % [rad]
    
    fields = {'footpad','robot','gait','freq','rot'};
    values = { foot    , robot , gait , freq , rotSeq};
    for i = 1:length(fields)
       info.(fields{i}) = values{i}; 
    end
    
    % Subfunctions
    function x = conv2mm(x,unit)
        switch unit
            case 'mm'
                x = x * 1e0;
            case 'cm'
                x = x * 1e1;
            case 'm'
                x = x * 1e3;
        end
    end

    function x = zeroing(x)
        x = x - x(1);
    end
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------
function [phi,theta,psi] = eulerAngles(rx,ry,rz,seq)
    % Initialize basic common parameters
    n     = length(rx);
    phi   = zeros(n,1);
    theta = zeros(n,1);
    psi   = zeros(n,1);
    
    for i = 1:n
        % Rotation matrix from the body to inertial frame
        C.x = basicRot('1',rx(i));
        C.y = basicRot('2',ry(i));
        C.z = basicRot('3',rz(i));
        C_ib = C.(seq(1))*C.(seq(2))*C.(seq(3));
        
        % Solve for the Euler Angles
        if C_ib(3,1) == -1
            % Not a unique solution
            psi(i)   = -atan2(-C_ib(2,3),C_ib(2,2));
            theta(i) = pi/2;
            phi(i)   = 0;
        elseif C_ib(3,1) == 1
            % Not a unique solution
            psi(i)   = atan2(-C_ib(2,3),C_ib(2,2));
            theta(i) = -pi/2;
            phi(i)   = 0;
        else
            % General unique solution
            psi(i)   = atan2(C_ib(2,1),C_ib(1,1));
            theta(i) = atan2(-C_ib(3,1),C_ib(1,1)/cos(psi(i)));
            phi(i)   = atan2(C_ib(3,2),C_ib(3,3));
        end
    end
    
    % Unwrap the angles for better turning representation
    phi   = unwrap(phi);
    theta = unwrap(theta);
    psi   = unwrap(psi);
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------
function [u,v,w,p,q,r] = estimateSpeeds(t,x,y,z,phi,theta,psi,fps,wpass)
    % Interpolate the recorded data to fill the missing frames
    dt = 1/fps;
    t2 = transpose(t(1)-dt : dt : t(end)+dt);
    data = [x y z phi theta psi];
    data = interp1(t,data,t2,'pchip');
    if all(wpass ~= [0 1])
        data = lowpass(data,wpass);
    end
    
    % Initialize basic common parameters
    n   = size(data,1);
    v_b = zeros(n,3);
    w_b = zeros(n,3);
    
    for i = 2:n-1
        % Rotation matrix from the inertial to body frame
        phi   = data(i,4);
        theta = data(i,5);
        psi   = data(i,6);
        C_ib  = basicRot('3',psi)*basicRot('2',theta)*basicRot('1',phi);
        C_bi  = transpose(C_ib);
        
        % Transformation matrix between euler rates and angular velocity
        cR = cos(phi); cP = cos(theta);
        sR = sin(phi); sP = sin(theta);
        
        J = [ 1	   sR*sP/cP    cR*sP/cP 
              0    cR         -sR       
              0    sR/cP       cR/cP   ];
        
        % Differentiate position and orientation data by central differencing
        p_i_dot  = transpose(data(i+1,1:3) - data(i-1,1:3)) / (2*dt);
        rpy_dot  = transpose(data(i+1,4:6) - data(i-1,4:6)) / (2*dt);
        
        % Estimate the linear and rotational velocity values
        v_b(i,:) = transpose(C_bi * p_i_dot);
        w_b(i,:) = transpose(J \ rpy_dot);
    end
    
    % Interpolate the estimated variables to the original time frame
    data = [v_b w_b];
    data = interp1(t2(2:end-1),data(2:end-1,:),t,'pchip');
    
    u = data(:,1);
    v = data(:,2);
    w = data(:,3);
    p = data(:,4);
    q = data(:,5);
    r = data(:,6);
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------
function R = basicRot(axis,a)
    % Basic rotation matrices about the primary axes by angle 'a' in radians
    switch lower(axis)
        case {'x','1',1}
            R = [ 1        0         0
                  0    cos(a)   -sin(a)
                  0    sin(a)    cos(a) ];
        
        case {'y','2',2}
            R = [ cos(a)    0    sin(a)
                      0     1        0
                 -sin(a)    0    cos(a) ];
        
        case {'z','3',3}
            R = [ cos(a)   -sin(a)    0
                  sin(a)    cos(a)    0
                      0         0     1 ];
    end
end
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------