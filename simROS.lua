local simROS = loadPlugin 'simROS';
(require 'simROS-typecheck')(simROS)

-- accept pure function where callback string is expected:

simROS.subscribe = wrap(simROS.subscribe, function(origFunc)
    return function(topicName, topicType, topicCallback, ...)
        return origFunc(topicName, topicType, reify(topicCallback), ...)
    end
end)

simROS.advertiseService = wrap(simROS.advertiseService, function(origFunc)
    return function(serviceName, serviceType, serviceCallback)
        return origFunc(serviceName, serviceType, reify(serviceCallback))
    end
end)

simROS.imageTransportSubscribe = wrap(simROS.imageTransportSubscribe, function(origFunc)
    return function(topicName, topicCallback, ...)
        return origFunc(topicName, reify(topicCallback), ...)
    end
end)

return simROS
