#pragma once
#include <memory>

namespace rcomponent
{

template<typename NodeType, typename... Args>
std::shared_ptr<NodeType> make_component(Args&&... args)
{
    auto ptr = std::shared_ptr<NodeType>(new NodeType(std::forward<Args>(args)...));
    ptr->Rcomponent::init();
    return ptr;
}

}