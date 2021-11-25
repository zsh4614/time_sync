#ifndef ANY_H_
#define ANY_H_

#include <memory>
#include <typeinfo>

class Any {
public:
    using Ptr = std::shared_ptr<Any>;

    Any() : content_(nullptr) {}

    template <typename ValueType>
    explicit Any(const ValueType& value) : content_(new Holder<ValueType>(value)) {}

    Any(const Any &other) : content_(other.content_ ? other.content_->Clone() : nullptr) {}

    ~Any() { delete content_; }

    const std::type_info& type_info() const { return content_ ? content_->type_info() : typeid(void); }

    template <typename ValueType>
    ValueType* AnyCast() { return content_ ? &(static_cast<Holder<ValueType>*>(content_)->held_) : nullptr; }

private:
    class PlaceHolder {
    public:
        virtual ~PlaceHolder() {}

        virtual const std::type_info& type_info() const = 0;

        virtual PlaceHolder* Clone() const = 0;
    };

    template <typename ValueType>
    class Holder : public PlaceHolder {
    public:
        explicit Holder(const ValueType& value) : held_(value) {}

        virtual const std::type_info& type_info() const { return typeid(ValueType); }

        virtual ~Holder() {}

        virtual PlaceHolder *Clone() const { return new Holder(held_); }

        ValueType held_;
    };

    PlaceHolder* content_;
};

#endif  // ANY_H_
