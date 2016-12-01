function [ imgOut ] = mirrorHorz( imgIn )
%creates left-right mirror image of imput image

    for i = 1:size(imgIn,2)
        imgOut(:,size(imgIn,2)-i+1,:) = imgIn(:,i,:);
    end
end

