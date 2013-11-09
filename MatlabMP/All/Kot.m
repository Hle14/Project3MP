function [] = Kot()
    x = 5;
    function [] = MyKeyPressFcn(varargin)
        disp(x);
    end

set(gcf, 'KeyPressFcn', {@MyKeyPressFcn});

end