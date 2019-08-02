% fid = fopen('PourIdeal1907241816');

function [A] = read_data(path)
    fid = fopen(path);
    % PourMasood3D_1907251440
    % PourMasood6D_1907251444
    tline = fgetl(fid);
    row = 1;
    A = [];
    while ischar(tline)
        if size(tline, 1) ~= 0
            C = textscan(tline,'%f');
            if size(C{1}, 1) == 6
                A = [A; C{1}(1) C{1}(2) C{1}(3) C{1}(4) C{1}(5) C{1}(6)];
            end
            tline = fgetl(fid);
        end
    end
%     x(1,:) = A(:,1);
%     y(1,:) = A(:,2);
%     z(1,:) = A(:,3);
end

% read_data('pose_data/PourMasood6D_1907251444')
% 
% fid = fopen('pose_data/PourMasood6D_1907251444');
% % PourMasood3D_1907251440
% % PourMasood6D_1907251444
% tline = fgetl(fid);
% row = 1;
% A = [];
% while ischar(tline)
%     if size(tline, 1) ~= 0
%         C = textscan(tline,'%f');
%         if size(C{1}, 1) == 6
%             A = [A; C{1}(1) C{1}(2) C{1}(3) C{1}(4) C{1}(5) C{1}(6)];
%         end
%         tline = fgetl(fid);
%     end
% end
% x(1,:) = A(:,1);
% y(1,:) = A(:,2);
% z(1,:) = A(:,3);
% createTrajectoryPlot3D(x, y, z, "2D Pour Masood");
% % scatter3(x,y,z);
% % plot3(x,y,z);
% fclose(fid);