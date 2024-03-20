close all; clc; clearvars -except conn;

device  = "PerchClimb";
service = "Custom";
if ~exist('conn','var')
    conn = ble(device);
end

servs = conn.Services;
chars = conn.Characteristics;
attrs = chars.Attributes;
uart  = find(servs.ServiceName == service);
uuidService = servs.ServiceUUID(uart);
iRX = find(cell2mat(cellfun(@(x) any(contains(x,"Notify")), ...
                            attrs,'UniformOutput',false)));
iTX = find(cell2mat(cellfun(@(x) any(contains(x,"Write")) , ...
                            attrs,'UniformOutput',false)));
uuidCharRX  = chars.CharacteristicUUID(iRX);
uuidCharTX  = chars.CharacteristicUUID(iTX);
rx = characteristic(conn, uuidService, uuidCharRX);
tx = characteristic(conn, uuidService, uuidCharTX);


MTU = 244; 
N   = 1000;
global COUNTER READINGS;
COUNTER = 0;
READINGS = zeros(N,MTU);
rx.DataAvailableFcn = @storeData;
tx.DataAvailableFcn = @processCommands;



function processCommands(src,~)
    global COUNTER READINGS;
    COUNTER = COUNTER + 1;
    READINGS(COUNTER,:) = read(src,'oldest');
    
end

function storeData(src,~)
    global COUNTER READINGS;
    COUNTER = COUNTER + 1;
    READINGS(COUNTER,:) = read(src,'oldest');
    
end
